#include "NotificationManager.h"
#include "Globals.h"
#include "GsmManager.h"

void NotificationManager::begin()
{
    loadQueue();

    preferences.begin(NVS_NAMESPACE, true);
    String stored = preferences.getString(NVS_HARVEST_KEY, "");
    preferences.end();
    strncpy(lastFiredHarvestEventId, stored.c_str(), sizeof(lastFiredHarvestEventId) - 1);

    uint8_t inUseCount = 0;
    for (uint8_t i = 0; i < NOTIFICATION_QUEUE_CAPACITY; i++)
    {
        if (queue[i].inUse) inUseCount++;
    }
    Serial.print("[NOTIFY] Queue loaded from NVS: ");
    Serial.print(inUseCount);
    Serial.println(" pending event(s)");
}

void NotificationManager::update()
{
    observeAlertTransitions();
    observeConnectivity();
    observeHarvestSchedule();
    updateSmsFanOut();
}

// --------------------------------------------------------------------
// Observation (read-only against AlertManager/SystemState/caches - never
// modifies any of them; mirrors FirebaseManager's own lastPublishedAlerts
// diff pattern already proven in this codebase).
// --------------------------------------------------------------------

void NotificationManager::observeAlertTransitions()
{
    if (!alertBaselineCaptured)
    {
        // Do not fire events for whatever state alerts already happen to be
        // in at boot - only genuinely NEW transitions after this baseline.
        lastObservedAlerts = alertState;
        alertBaselineCaptured = true;
        return;
    }

    // Cloud-replay ownership: while online, /alerts already reaches Firestore
    // directly via onAlertUpdated (immediate FCM + history) the instant
    // writeAlerts() publishes the same transition - queuing these same four
    // types for cloud replay too produced a second, differently-worded
    // Firestore history document under a different eventId scheme with
    // nothing to deduplicate the two (see the task report's duplicate-history
    // audit). Cloud replay is therefore only needed as the offline fallback,
    // matching the ownership DEVICE_UNREACHABLE/HARVEST_DUE already use below
    // (backend owns online history for both cases - its own independent
    // presence detection there, the direct /alerts watch here). SMS
    // eligibility is untouched: a farmer's phone connectivity is independent
    // of the device's, so the redundant delivery channel stays unconditional.
    const bool cloudUp = systemState.wifiConnected && systemState.firebaseConnected;

    if (alertState.lowWater && !lastObservedAlerts.lowWater)
    {
        enqueueEvent(NotificationEventType::LOW_WATER, NotificationSeverity::SEV_HIGH,
                     "Low Reservoir", "Water level dropped below the refill threshold.",
                     String(millis()), true, !cloudUp);
    }
    if (alertState.waterTempOutOfRange && !lastObservedAlerts.waterTempOutOfRange)
    {
        enqueueEvent(NotificationEventType::HIGH_WATER_TEMP, NotificationSeverity::SEV_HIGH,
                     "High Water Temperature", "Water temperature exceeded the configured limit.",
                     String(millis()), true, !cloudUp);
    }
    if (alertState.highTemperature && !lastObservedAlerts.highTemperature)
    {
        enqueueEvent(NotificationEventType::HIGH_AIR_TEMP, NotificationSeverity::SEV_HIGH,
                     "High Air Temperature", "Air temperature exceeded the configured limit.",
                     String(millis()), true, !cloudUp);
    }
    if (alertState.sensorFault && !lastObservedAlerts.sensorFault)
    {
        enqueueEvent(NotificationEventType::SENSOR_FAULT, NotificationSeverity::SEV_CRITICAL,
                     "Sensor Fault", "One or more sensors are reporting invalid readings.",
                     String(millis()), true, !cloudUp);
    }

    lastObservedAlerts = alertState;
}

void NotificationManager::observeConnectivity()
{
    bool cloudUp = systemState.wifiConnected && systemState.firebaseConnected;

    if (!cloudUp)
    {
        if (!cloudWasDown)
        {
            cloudDownSinceMillis = millis();
            cloudWasDown = true;
        }
        else if (!deviceUnreachableEpisodeActive &&
                 millis() - cloudDownSinceMillis >= OFFLINE_EPISODE_THRESHOLD_MS)
        {
            deviceUnreachableEpisodeActive = true;
            // SMS-only (queuedForCloud=false): backend already owns Device
            // Unreachable/Back Online history via its own independent RTDB
            // presence detection, which needs no firmware action once
            // connectivity actually drops. Replaying this to Firestore too
            // would risk a second, differently-worded history entry for the
            // same outage - see the ownership decision in the task report.
            enqueueEvent(NotificationEventType::DEVICE_UNREACHABLE, NotificationSeverity::SEV_CRITICAL,
                         "Device Unreachable", "Basilience device lost connectivity.",
                         String(millis()), true, false);
        }
    }
    else
    {
        // Recovered: close the episode. Local automation is never touched by
        // this transition - only notification bookkeeping.
        cloudWasDown = false;
        deviceUnreachableEpisodeActive = false;
    }
}

void NotificationManager::observeHarvestSchedule()
{
    if (!harvestScheduleCache.isActive()) return;
    if (!rtcManager.hasValidTime()) return; // cannot safely judge "due" without a trustworthy clock

    uint32_t nextHarvestAt = harvestScheduleCache.getNextHarvestAtEpoch();
    if (nextHarvestAt == 0) return;

    uint32_t nowEpoch = rtcManager.getEpochTime();
    if (nowEpoch < nextHarvestAt) return; // not due yet

    String cycleId = harvestScheduleCache.getCycleId();
    char identity[48];
    snprintf(identity, sizeof(identity), "%s_%lu", cycleId.c_str(), (unsigned long)nextHarvestAt);

    char candidateEventId[64];
    snprintf(candidateEventId, sizeof(candidateEventId), "%s_%s",
             notificationEventTypeName(NotificationEventType::HARVEST_DUE), identity);

    if (strcmp(lastFiredHarvestEventId, candidateEventId) == 0) return; // already fired for this scheduled occurrence
    if (findQueueSlot(candidateEventId) >= 0) return; // still queued from a previous tick

    char message[160];
    snprintf(message, sizeof(message), "Harvest for Cycle #%d is due today.",
             harvestScheduleCache.getCycleNumber());

    // SMS-only (queuedForCloud=false): backend already owns Harvest Due
    // history via its own independent hourly evaluateHarvestReminders cron,
    // which scans Firestore's nextHarvestDate directly and needs no
    // firmware action - it fires whether or not this device is online.
    // Replaying this to Firestore too would risk a second, differently-
    // worded history entry for the same scheduled harvest.
    enqueueEvent(NotificationEventType::HARVEST_DUE, NotificationSeverity::SEV_MEDIUM,
                 "Harvest Due", message, String(identity), true, false);

    strncpy(lastFiredHarvestEventId, candidateEventId, sizeof(lastFiredHarvestEventId) - 1);
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putString(NVS_HARVEST_KEY, String(lastFiredHarvestEventId));
    preferences.end();
}

// --------------------------------------------------------------------
// Durable queue
// --------------------------------------------------------------------

void NotificationManager::enqueueEvent(NotificationEventType type, NotificationSeverity severity,
                                        const char* title, const char* message,
                                        const String& episodeIdentity, bool smsEligible, bool queuedForCloud)
{
    char eventId[40];
    snprintf(eventId, sizeof(eventId), "%s_%s", notificationEventTypeName(type), episodeIdentity.c_str());

    if (findQueueSlot(eventId) >= 0) return; // defensively idempotent

    int slot = findFreeOrEvictableSlot();
    if (slot < 0)
    {
        Serial.println("[QUEUE] Full - unable to add event, even after eviction attempt");
        return;
    }

    NotificationEvent event; // default-constructed
    event.inUse = true;
    strncpy(event.eventId, eventId, sizeof(event.eventId) - 1);
    event.type = type;
    event.severity = severity;
    strncpy(event.title, title, sizeof(event.title) - 1);
    strncpy(event.message, message, sizeof(event.message) - 1);

    if (rtcManager.hasValidTime())
    {
        event.occurredAtEpoch = rtcManager.getEpochTime();
        event.timestampValid = true;
    }

    event.smsEligible = smsEligible;
    event.smsStatus = SmsDeliveryStatus::PENDING;
    event.queuedForCloud = queuedForCloud;
    event.cloudStatus = queuedForCloud ? CloudReplayStatus::PENDING : CloudReplayStatus::NOT_QUEUED;

    queue[slot] = event;
    persistQueue();

    Serial.print("[NOTIFY] Queued ");
    Serial.print(notificationEventTypeName(type));
    Serial.print(" ");
    Serial.println(eventId);
}

int NotificationManager::findQueueSlot(const char* eventId)
{
    for (int i = 0; i < NOTIFICATION_QUEUE_CAPACITY; i++)
    {
        if (queue[i].inUse && strcmp(queue[i].eventId, eventId) == 0) return i;
    }
    return -1;
}

int NotificationManager::findFreeOrEvictableSlot()
{
    for (int i = 0; i < NOTIFICATION_QUEUE_CAPACITY; i++)
    {
        if (!queue[i].inUse) return i;
    }

    // Full: prefer preserving HIGH/CRITICAL events - evict the first
    // LOW/MEDIUM-severity entry found (an approximate "oldest low-severity
    // first" scan; the queue has no separate insertion-order index, kept
    // deliberately small per the task's "if safe and small" guidance). If
    // nothing qualifies, refuse the new event rather than evict something
    // important - bounded FIFO with a diagnostic, not silent corruption.
    for (int i = 0; i < NOTIFICATION_QUEUE_CAPACITY; i++)
    {
        if (queue[i].severity == NotificationSeverity::SEV_LOW ||
            queue[i].severity == NotificationSeverity::SEV_MEDIUM)
        {
            Serial.print("[QUEUE] Full - evicting low-priority ");
            Serial.println(queue[i].eventId);
            return i;
        }
    }

    Serial.println("[QUEUE] Full - no low-priority event to evict, new event dropped");
    return -1;
}

void NotificationManager::loadQueue()
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

void NotificationManager::persistQueue()
{
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putBytes(NVS_QUEUE_KEY, queue, sizeof(queue));
    preferences.end();
}

// --------------------------------------------------------------------
// SMS fan-out: one event, one recipient at a time, fully non-blocking.
// GsmManager itself has no multi-recipient policy - that lives entirely
// here, matching the layering the GSM foundation task established.
// --------------------------------------------------------------------

void NotificationManager::updateSmsFanOut()
{
    if (!smsFanOutActive)
    {
        startSmsFanOutIfIdle();
        return;
    }

    if (fanOutStep == FanOutStep::RECIPIENT_DELAY)
    {
        if (millis() < smsFanOutRetryAt) return;
        attemptCurrentRecipient();
        return;
    }

    // AWAITING_RESULT
    if (gsmManager.isBusy()) return;

    NotificationEvent& event = queue[smsFanOutEventIndex];
    GsmManager::SendResult result = gsmManager.getLastResult();
    uint8_t r = smsFanOutRecipientIndex;

    if (result == GsmManager::SendResult::SUCCESS)
    {
        event.recipientState[r] = (uint8_t)RecipientSmsState::SENT;
        Serial.print("[SMS] Delivered ");
        Serial.println(event.eventId);
    }
    else if (result == GsmManager::SendResult::INVALID_NUMBER)
    {
        // Bounded policy: no retry for a structurally invalid number.
        event.recipientState[r] = (uint8_t)RecipientSmsState::FAILED_NO_RETRY;
    }
    else if (result == GsmManager::SendResult::MODULE_NOT_READY ||
             result == GsmManager::SendResult::SIM_NOT_READY ||
             result == GsmManager::SendResult::NOT_REGISTERED)
    {
        // Cellular itself unavailable - defer the WHOLE event rather than
        // mark this one recipient failed; resumed later once ready (see
        // startSmsFanOutIfIdle()'s freshness-gated resume).
        event.smsStatus = SmsDeliveryStatus::DEFERRED;
        Serial.println("[SMS] Deferred: cellular unavailable");
        fanOutStep = FanOutStep::IDLE;
        smsFanOutActive = false;
        persistQueue();
        return;
    }
    else // TIMEOUT / ERROR / NONE
    {
        if (event.recipientRetryCount[r] < 1)
        {
            // Bounded policy: at most one retry for TIMEOUT/ERROR.
            event.recipientRetryCount[r]++;
            fanOutStep = FanOutStep::RECIPIENT_DELAY;
            smsFanOutRetryAt = millis() + SMS_RETRY_DELAY_MS;
            return;
        }
        event.recipientState[r] = (uint8_t)RecipientSmsState::FAILED_RETRY_EXHAUSTED;
    }

    smsFanOutRecipientIndex++;
    attemptCurrentRecipient();
}

void NotificationManager::startSmsFanOutIfIdle()
{
    for (int i = 0; i < NOTIFICATION_QUEUE_CAPACITY; i++)
    {
        NotificationEvent& event = queue[i];
        if (!event.inUse || !event.smsEligible) continue;
        if (event.smsStatus != SmsDeliveryStatus::PENDING &&
            event.smsStatus != SmsDeliveryStatus::DEFERRED) continue;

        if (event.smsStatus == SmsDeliveryStatus::DEFERRED)
        {
            if (!gsmManager.isReady()) continue; // still not registered; try a later tick

            unsigned long freshnessMs = (event.type == NotificationEventType::HARVEST_DUE)
                    ? HARVEST_DUE_SMS_FRESHNESS_MS : GENERAL_SMS_FRESHNESS_MS;
            if (event.timestampValid && rtcManager.hasValidTime())
            {
                uint32_t nowEpoch = rtcManager.getEpochTime();
                uint32_t ageSeconds = nowEpoch > event.occurredAtEpoch ? nowEpoch - event.occurredAtEpoch : 0;
                if ((unsigned long)ageSeconds * 1000UL > freshnessMs)
                {
                    // Do not send a very old warning merely because cellular
                    // recovered later - the event still replays to history
                    // with its original occurredAt regardless.
                    event.smsStatus = SmsDeliveryStatus::FAILED;
                    Serial.print("[SMS] Expired before cellular recovered: ");
                    Serial.println(event.eventId);
                    persistQueue();
                    continue;
                }
            }
        }
        else if (!gsmManager.isReady())
        {
            continue;
        }

        smsFanOutActive = true;
        smsFanOutEventIndex = (uint8_t)i;
        smsFanOutRecipientIndex = 0;
        event.smsStatus = SmsDeliveryStatus::IN_PROGRESS;
        attemptCurrentRecipient();
        return; // only start one event's fan-out per idle check
    }
}

void NotificationManager::attemptCurrentRecipient()
{
    NotificationEvent& event = queue[smsFanOutEventIndex];

    // Skip a recipient index whose canonical phone duplicates one already
    // attempted for THIS event, so a number shared by two accounts (e.g.
    // Admin and Personnel) only ever receives one physical SMS per event.
    while (smsFanOutRecipientIndex < smsRecipientCache.recipientCount())
    {
        String phone = smsRecipientCache.recipientAt(smsFanOutRecipientIndex);
        bool alreadyAttempted = false;
        for (uint8_t j = 0; j < smsFanOutRecipientIndex; j++)
        {
            if (smsRecipientCache.recipientAt(j) == phone) { alreadyAttempted = true; break; }
        }
        if (!alreadyAttempted) break;
        smsFanOutRecipientIndex++;
    }

    if (smsFanOutRecipientIndex >= smsRecipientCache.recipientCount())
    {
        finishSmsFanOut();
        return;
    }

    String phone = smsRecipientCache.recipientAt(smsFanOutRecipientIndex);
    String body = buildSmsBody(event);

    Serial.print("[SMS] Sending ");
    Serial.print(event.eventId);
    Serial.print(" to ");
    Serial.println(GsmManager::maskPhoneNumber(phone));

    if (gsmManager.sendSms(phone, body))
    {
        fanOutStep = FanOutStep::AWAITING_RESULT;
    }
    else
    {
        // Rejected immediately (module not ready/busy) - treat as a deferred
        // whole-event outcome rather than spin retrying within this tick.
        event.smsStatus = SmsDeliveryStatus::DEFERRED;
        fanOutStep = FanOutStep::IDLE;
        smsFanOutActive = false;
        persistQueue();
    }
}

void NotificationManager::finishSmsFanOut()
{
    NotificationEvent& event = queue[smsFanOutEventIndex];
    uint8_t total = smsRecipientCache.recipientCount();
    uint8_t sent = 0;
    bool anyAttempted = false;

    for (uint8_t i = 0; i < total; i++)
    {
        if (event.recipientState[i] == (uint8_t)RecipientSmsState::SENT) sent++;
        if (event.recipientState[i] != (uint8_t)RecipientSmsState::NOT_ATTEMPTED) anyAttempted = true;
    }

    if (total == 0 || !anyAttempted) event.smsStatus = SmsDeliveryStatus::FAILED;
    else if (sent == total) event.smsStatus = SmsDeliveryStatus::DELIVERED;
    else if (sent > 0) event.smsStatus = SmsDeliveryStatus::PARTIAL;
    else event.smsStatus = SmsDeliveryStatus::FAILED;

    fanOutStep = FanOutStep::IDLE;
    smsFanOutActive = false;
    persistQueue();
}

String NotificationManager::buildSmsBody(const NotificationEvent& event) const
{
    // "Basilience: <title> - <message>", GSM-7-friendly (no emoji/curly
    // punctuation), trimmed to stay within one ~160-char SMS segment. The
    // same title/message is used verbatim for the later Firestore replay -
    // wording is authored once here, never regenerated after reconnect.
    String body = "Basilience: ";
    body += event.title;
    body += " - ";
    body += event.message;
    if (body.length() > 155) body = body.substring(0, 155);
    return body;
}

// --------------------------------------------------------------------
// Cloud replay integration (called from FirebaseManager's job cursor)
// --------------------------------------------------------------------

bool NotificationManager::getNextCloudReplayEvent(NotificationEvent& out)
{
    if (cloudReplayInFlightEventId[0] != '\0')
    {
        int slot = findQueueSlot(cloudReplayInFlightEventId);
        if (slot >= 0)
        {
            out = queue[slot];
            return true;
        }
        cloudReplayInFlightEventId[0] = '\0'; // was removed/acked elsewhere; fall through
    }

    for (int i = 0; i < NOTIFICATION_QUEUE_CAPACITY; i++)
    {
        if (queue[i].inUse && queue[i].queuedForCloud &&
            queue[i].cloudStatus == CloudReplayStatus::PENDING)
        {
            strncpy(cloudReplayInFlightEventId, queue[i].eventId, sizeof(cloudReplayInFlightEventId) - 1);
            cloudReplaySubmittedAtMillis = 0;
            out = queue[i];
            return true;
        }
    }
    return false;
}

bool NotificationManager::isCloudReplayStale(const char* eventId, unsigned long staleAfterMs)
{
    if (strcmp(cloudReplayInFlightEventId, eventId) != 0) return true; // nothing submitted for it yet
    if (cloudReplaySubmittedAtMillis == 0) return true;
    return millis() - cloudReplaySubmittedAtMillis >= staleAfterMs;
}

void NotificationManager::markCloudReplaySubmitted(const char* eventId)
{
    int slot = findQueueSlot(eventId);
    if (slot < 0) return;
    queue[slot].cloudStatus = CloudReplayStatus::IN_PROGRESS;
    cloudReplaySubmittedAtMillis = millis();
    persistQueue();
    Serial.print("[QUEUE] Replaying ");
    Serial.println(eventId);
}

void NotificationManager::markCloudReplayAcked(const char* eventId)
{
    int slot = findQueueSlot(eventId);
    if (slot < 0) return;
    Serial.print("[QUEUE] Cloud ack ");
    Serial.println(eventId);
    queue[slot] = NotificationEvent(); // clears inUse too
    cloudReplayInFlightEventId[0] = '\0';
    persistQueue();
}
