#ifndef FOGGING_EVENT_QUEUE_H
#define FOGGING_EVENT_QUEUE_H

#include <Arduino.h>
#include <Preferences.h>
#include "FoggingQueueTypes.h"

// Durable, offline-capable history of CONFIRMED fogger ON/OFF transitions -
// history correctness only. Does not decide when the fogger runs (that
// stays entirely in AutomationManager/ActuatorManager); this class only
// observes the already-confirmed result and makes sure a record of it
// reaches Firestore exactly once, whether the device was online at the
// moment of the transition or not.
//
// Deliberately not a copy of NotificationManager: no SMS fan-out, no
// severity/eviction-by-priority, no free-text title/message. Fogging
// transitions are frequent and homogeneous, so this is a strict oldest-first
// FIFO with compact coded fields (see FoggingQueueTypes.h).
class FoggingEventQueue
{
public:
    void begin();

    // Call only from the exact point a FOGGER transition is CONFIRMED
    // (ActuatorManager::update(), where status.running actually flips) -
    // never from request submission. isRunning=true records an OFF->ON
    // transition, false records ON->OFF.
    void recordConfirmedTransition(bool isRunning, const String& source,
                                    const String& strategy, const String& reason);

    // eventId of the most recently recorded transition this boot, or ""
    // before the first one. Stamped onto actuatorStatus/fogger by
    // FirebaseManager::writeActuators() as the single-producer compatibility
    // marker so the legacy state-trigger Cloud Function knows to defer to
    // this queue's own replay path instead of double-logging.
    String getLastEventId() const;

    // --- Cloud replay integration, called from FirebaseManager's optional
    //     job cursor - one job slot, one event in flight at a time, same
    //     shape as NotificationManager's cloud replay integration so it
    //     shares the existing health/backoff/starvation-avoidance behavior
    //     for free (see runOneOptionalFirebaseJob). ---
    bool getNextReplayEvent(FoggingQueueEvent& outEvent, String& outEventId);
    bool isReplayStale(const String& eventId, unsigned long staleAfterMs);
    void markReplaySubmitted(const String& eventId);
    // Removes the event from the durable queue - only call after confirmed
    // authoritative Firestore persistence (the Cloud Function's ack).
    void markReplayAcked(const String& eventId);

private:
    static constexpr const char* NVS_NAMESPACE = "foggingq";
    static constexpr const char* NVS_QUEUE_KEY = "queue";
    static constexpr const char* NVS_BOOTID_KEY = "bootId";
    // Persisted independently of the queue blob so reboot-recovery still
    // works even after every queued event has already been acked/removed.
    static constexpr const char* NVS_LASTRUN_KEY = "lastRunning";

    Preferences preferences;
    FoggingQueueEvent queue[FOGGING_QUEUE_CAPACITY];

    uint16_t bootId = 0;
    uint16_t nextSequence = 0;

    char lastEventIdBuf[24] = {0};

    // In-RAM only; safe to reset on reboot since every (re)submission writes
    // the full, idempotent event content again (same reasoning as
    // NotificationManager's cloudReplayInFlightEventId).
    char replayInFlightEventId[24] = {0};
    unsigned long replaySubmittedAtMillis = 0;

    void enqueue(FoggingEventType type, FoggingSourceCode source,
                 FoggingStrategyCode strategy, FoggingReasonCode reason);

    void buildEventId(char* out, size_t outSize, uint16_t boot, uint16_t seq) const;
    int findSlotByEventId(const char* eventId);
    // Returns the in-use slot with the smallest (bootId, sequence) - i.e. the
    // chronologically oldest event - or -1 if the queue is empty.
    int findOldestInUseSlot();
    int findFreeSlot();

    void loadQueue();
    void persistQueue();
};

#endif
