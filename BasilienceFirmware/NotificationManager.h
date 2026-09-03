#ifndef NOTIFICATION_MANAGER_H
#define NOTIFICATION_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>
#include "NotificationTypes.h"
#include "Types.h"

// Orchestrates offline-capable firmware notifications end to end:
//   - observes existing AlertManager/connectivity/harvest-schedule state
//     (never modifies any of it) to detect new episodes worth notifying
//   - maintains a bounded, NVS-persisted durable queue of canonical events
//   - drives a one-recipient-at-a-time SMS fan-out through GsmManager
//   - hands queued events to FirebaseManager's existing low-starvation job
//     cursor for cloud replay, one at a time, removed only after ack
//
// update() is non-blocking and safe to call every loop() iteration
// unconditionally (same treatment as GsmManager) - cultivation control is
// never delayed by anything in this class.
class NotificationManager
{
public:
    void begin();
    void update();

    // --- Cloud replay integration point, called from FirebaseManager's
    //     existing optional-job cursor (one job slot, one event at a time). ---

    // Fills `out` with the event currently due for cloud replay (the same
    // in-flight event on repeated calls until acked/removed). Returns false
    // if nothing is waiting.
    bool getNextCloudReplayEvent(NotificationEvent& out);
    // True if nothing has been submitted yet for eventId, or the previous
    // submission is older than staleAfterMs (worth resubmitting rather than
    // just polling again).
    bool isCloudReplayStale(const char* eventId, unsigned long staleAfterMs);
    void markCloudReplaySubmitted(const char* eventId);
    // Removes the event from the durable queue - only call after confirmed
    // authoritative Firebase persistence (the Cloud Function's ack).
    void markCloudReplayAcked(const char* eventId);

private:
    enum class FanOutStep : uint8_t { IDLE, AWAITING_RESULT, RECIPIENT_DELAY };

    static constexpr const char* NVS_NAMESPACE = "notifyq";
    static constexpr const char* NVS_QUEUE_KEY = "queue";
    static constexpr const char* NVS_HARVEST_KEY = "lastHarvestId";

    // A sustained cloud outage this long constitutes a real offline episode
    // worth a DEVICE_UNREACHABLE SMS - no existing threshold covers this
    // (the firmware audit found none), chosen new: long enough that a brief
    // Wi-Fi wobble never fires it, short enough to still be a timely alert.
    static constexpr unsigned long OFFLINE_EPISODE_THRESHOLD_MS = 2UL * 60UL * 1000UL;
    static constexpr unsigned long SMS_RETRY_DELAY_MS = 30UL * 1000UL;
    static constexpr unsigned long CLOUD_REPLAY_RESUBMIT_MS = 5UL * 60UL * 1000UL;
    // Freshness policy (Part S): a deferred SMS is only attempted once
    // cellular recovers if the event is still this fresh - otherwise it is
    // marked FAILED (no stale multi-day-old warning is ever sent). Harvest
    // reminders tolerate a longer window since "due today" is still useful
    // information a day later; ordinary alerts do not.
    static constexpr unsigned long GENERAL_SMS_FRESHNESS_MS = 6UL * 60UL * 60UL * 1000UL;       // 6 hours
    static constexpr unsigned long HARVEST_DUE_SMS_FRESHNESS_MS = 24UL * 60UL * 60UL * 1000UL;  // 24 hours
    // A PENDING event whose GSM module never becomes ready (no SIM800L
    // wired, or it never registers) previously waited forever with no
    // bound at all - this is the timeout that gives up on ATTEMPTING SMS in
    // that case (distinct from GENERAL_SMS_FRESHNESS_MS above, which governs
    // whether an already-DEFERRED send is still worth attempting once GSM
    // does recover). 5 minutes is generous enough not to punish a real
    // SIM800L's normal power-on/network-registration delay.
    static constexpr unsigned long SMS_START_TIMEOUT_MS = 5UL * 60UL * 1000UL;

    Preferences preferences;
    NotificationEvent queue[NOTIFICATION_QUEUE_CAPACITY];

    // Mirrors FirebaseManager's own lastPublishedAlerts diff pattern (already
    // proven in this codebase) so alert transitions can be OBSERVED without
    // touching AlertManager itself.
    AlertState lastObservedAlerts;
    bool alertBaselineCaptured = false;

    bool alertNotificationAllowed(NotificationEventType type) const;

    bool deviceUnreachableEpisodeActive = false;
    bool cloudWasDown = false;
    unsigned long cloudDownSinceMillis = 0;

    char lastFiredHarvestEventId[64] = {0};

    bool smsFanOutActive = false;
    FanOutStep fanOutStep = FanOutStep::IDLE;
    uint8_t smsFanOutEventIndex = 0;
    uint8_t smsFanOutRecipientIndex = 0;
    unsigned long smsFanOutRetryAt = 0;

    // In-RAM only; safe to reset on reboot since every (re)submission writes
    // the full, idempotent event content again.
    char cloudReplayInFlightEventId[40] = {0};
    unsigned long cloudReplaySubmittedAtMillis = 0;

    void observeAlertTransitions();
    void observeConnectivity();
    void observeHarvestSchedule();

    void enqueueEvent(NotificationEventType type, NotificationSeverity severity,
                       const char* title, const char* message,
                       const String& episodeIdentity, bool smsEligible, bool queuedForCloud);

    void updateSmsFanOut();
    void startSmsFanOutIfIdle();
    void attemptCurrentRecipient();
    void finishSmsFanOut();

    String buildSmsBody(const NotificationEvent& event) const;

    int findQueueSlot(const char* eventId);
    int findFreeOrEvictableSlot();
    void reapSettledSlots();
    void loadQueue();
    void persistQueue();
};

#endif
