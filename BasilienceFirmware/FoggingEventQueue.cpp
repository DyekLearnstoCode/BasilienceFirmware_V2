#include "FoggingEventQueue.h"
#include "Globals.h"

void FoggingEventQueue::begin()
{
    loadQueue();

    preferences.begin(NVS_NAMESPACE, false);
    bootId = preferences.getUShort(NVS_BOOTID_KEY, 0) + 1;
    preferences.putUShort(NVS_BOOTID_KEY, bootId);
    bool lastRunning = preferences.getBool(NVS_LASTRUN_KEY, false);
    preferences.end();

    nextSequence = 0;

    uint16_t inUseCount = 0;
    for (uint16_t i = 0; i < FOGGING_QUEUE_CAPACITY; i++)
    {
        if (queue[i].inUse) inUseCount++;
    }
    Serial.print("[FOGQ] Queue loaded from NVS: ");
    Serial.print(inUseCount);
    Serial.print(" pending event(s), bootId=");
    Serial.println(bootId);

    // Reboot-recovery: the persisted "was the fogger last confirmed ON"
    // flag survived across this reboot, but ActuatorManager::begin() (which
    // already ran before this call - see setup()) always initializes every
    // actuator, including FOGGER, to physically OFF. If the last thing this
    // queue recorded was ON, that ON session never got a matching OFF -
    // either from a clean shutdown or from ActuatorManager's own STOPPING
    // path - so one is appended now. Boot/recovery time only, per the task:
    // the exact physical stop time (somewhere between the crash/power-loss
    // and this boot) is unknowable and must not be fabricated.
    if (lastRunning)
    {
        Serial.println("[FOGQ] Reboot recovery: last known state was ON - recording closeout OFF");
        enqueue(FoggingEventType::OFF, FoggingSourceCode::REBOOT_RECOVERY,
                FoggingStrategyCode::NONE, FoggingReasonCode::REBOOT_RECOVERY);
    }
}

void FoggingEventQueue::recordConfirmedTransition(bool isRunning, const String& source,
                                                    const String& strategy, const String& reason)
{
    FoggingSourceCode sourceCode = FoggingSourceCode::AUTOMATIC;
    if (source == "manual") sourceCode = FoggingSourceCode::MANUAL;
    else if (source == "android") sourceCode = FoggingSourceCode::ANDROID;

    FoggingStrategyCode strategyCode = FoggingStrategyCode::NONE;
    if (strategy == "hot") strategyCode = FoggingStrategyCode::HOT;
    else if (strategy == "cold") strategyCode = FoggingStrategyCode::COLD;
    else if (strategy == "startup") strategyCode = FoggingStrategyCode::STARTUP;

    FoggingReasonCode reasonCode = FoggingReasonCode::NONE;
    if (reason == "Manual stop") reasonCode = FoggingReasonCode::MANUAL_STOP;
    else if (reason == "Safety limit: manual actuator running too long") reasonCode = FoggingReasonCode::SAFETY_TIMEOUT;
    else if (reason.length() > 0) reasonCode = FoggingReasonCode::SAFETY_SUSPENDED; // any other non-empty reason: suspended by safety/validation

    enqueue(isRunning ? FoggingEventType::ON : FoggingEventType::OFF, sourceCode, strategyCode, reasonCode);

    // Persisted separately from the queue blob so reboot-recovery still
    // works after this event has already been replayed and removed.
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putBool(NVS_LASTRUN_KEY, isRunning);
    preferences.end();
}

void FoggingEventQueue::enqueue(FoggingEventType type, FoggingSourceCode source,
                                 FoggingStrategyCode strategy, FoggingReasonCode reason)
{
    int slot = findFreeSlot();
    if (slot < 0)
    {
        // Bounded FIFO: full queue means every slot is still unacked -
        // dropping the oldest is the documented overflow policy (never
        // block cultivation to wait for cloud catch-up).
        slot = findOldestInUseSlot();
        if (slot < 0) return; // capacity is 0 - unreachable in practice
        Serial.print("[FOGQ] Full - dropping oldest unacked event, boot=");
        Serial.print(queue[slot].bootId);
        Serial.print(" seq=");
        Serial.println(queue[slot].sequence);
    }

    FoggingQueueEvent event; // default-constructed
    event.inUse = true;
    event.eventType = (uint8_t)type;
    event.sourceCode = (uint8_t)source;
    event.strategyCode = (uint8_t)strategy;
    event.reasonCode = (uint8_t)reason;
    event.bootId = bootId;
    event.sequence = nextSequence++;

    if (rtcManager.hasValidTime())
    {
        event.occurredAtEpoch = rtcManager.getEpochTime();
        event.timestampValid = true;
    }

    queue[slot] = event;
    persistQueue();

    buildEventId(lastEventIdBuf, sizeof(lastEventIdBuf), event.bootId, event.sequence);

    Serial.print("[FOGQ] Recorded ");
    Serial.print(type == FoggingEventType::ON ? "ON " : "OFF ");
    Serial.println(lastEventIdBuf);
}

String FoggingEventQueue::getLastEventId() const
{
    return String(lastEventIdBuf);
}

void FoggingEventQueue::buildEventId(char* out, size_t outSize, uint16_t boot, uint16_t seq) const
{
    snprintf(out, outSize, "fog_b%u_s%u", (unsigned)boot, (unsigned)seq);
}

int FoggingEventQueue::findSlotByEventId(const char* eventId)
{
    char candidate[24];
    for (int i = 0; i < FOGGING_QUEUE_CAPACITY; i++)
    {
        if (!queue[i].inUse) continue;
        buildEventId(candidate, sizeof(candidate), queue[i].bootId, queue[i].sequence);
        if (strcmp(candidate, eventId) == 0) return i;
    }
    return -1;
}

int FoggingEventQueue::findOldestInUseSlot()
{
    int oldest = -1;
    for (int i = 0; i < FOGGING_QUEUE_CAPACITY; i++)
    {
        if (!queue[i].inUse) continue;
        if (oldest < 0 ||
            queue[i].bootId < queue[oldest].bootId ||
            (queue[i].bootId == queue[oldest].bootId && queue[i].sequence < queue[oldest].sequence))
        {
            oldest = i;
        }
    }
    return oldest;
}

int FoggingEventQueue::findFreeSlot()
{
    for (int i = 0; i < FOGGING_QUEUE_CAPACITY; i++)
    {
        if (!queue[i].inUse) return i;
    }
    return -1;
}

void FoggingEventQueue::loadQueue()
{
    preferences.begin(NVS_NAMESPACE, true);
    size_t expectedSize = sizeof(queue);
    size_t actualSize = preferences.getBytesLength(NVS_QUEUE_KEY);
    if (actualSize == expectedSize)
    {
        preferences.getBytes(NVS_QUEUE_KEY, queue, expectedSize);
    }
    // Any mismatch (first boot / no prior blob / a struct-layout change)
    // leaves `queue` at its default-constructed, all-empty state rather than
    // reading garbage.
    preferences.end();
}

void FoggingEventQueue::persistQueue()
{
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putBytes(NVS_QUEUE_KEY, queue, sizeof(queue));
    preferences.end();
}

// --------------------------------------------------------------------
// Cloud replay integration (called from FirebaseManager's job cursor)
// --------------------------------------------------------------------

bool FoggingEventQueue::getNextReplayEvent(FoggingQueueEvent& outEvent, String& outEventId)
{
    if (replayInFlightEventId[0] != '\0')
    {
        int slot = findSlotByEventId(replayInFlightEventId);
        if (slot >= 0)
        {
            outEvent = queue[slot];
            outEventId = replayInFlightEventId;
            return true;
        }
        replayInFlightEventId[0] = '\0'; // was acked/removed elsewhere; fall through
    }

    // Oldest-first: never pick an arbitrary pending event while an older one
    // still waits, so replay order matches the order transitions actually
    // happened in.
    int slot = findOldestInUseSlot();
    if (slot < 0) return false;

    char eventId[24];
    buildEventId(eventId, sizeof(eventId), queue[slot].bootId, queue[slot].sequence);
    strncpy(replayInFlightEventId, eventId, sizeof(replayInFlightEventId) - 1);
    replaySubmittedAtMillis = 0;

    outEvent = queue[slot];
    outEventId = eventId;
    return true;
}

bool FoggingEventQueue::isReplayStale(const String& eventId, unsigned long staleAfterMs)
{
    if (strcmp(replayInFlightEventId, eventId.c_str()) != 0) return true; // nothing submitted for it yet
    if (replaySubmittedAtMillis == 0) return true;
    return millis() - replaySubmittedAtMillis >= staleAfterMs;
}

void FoggingEventQueue::markReplaySubmitted(const String& eventId)
{
    if (strcmp(replayInFlightEventId, eventId.c_str()) != 0) return;
    replaySubmittedAtMillis = millis();
    if (debugManager.shouldPrintDebug(DebugCategory::NOTIFICATION))
    {
        Serial.print("[FOGQ] Replaying ");
        Serial.println(eventId);
    }
}

void FoggingEventQueue::markReplayAcked(const String& eventId)
{
    int slot = findSlotByEventId(eventId.c_str());
    if (slot < 0) return;
    if (debugManager.shouldPrintDebug(DebugCategory::NOTIFICATION))
    {
        Serial.print("[FOGQ] Cloud ack ");
        Serial.println(eventId);
    }
    queue[slot] = FoggingQueueEvent(); // clears inUse too
    replayInFlightEventId[0] = '\0';
    persistQueue();
}
