from navigation_mission_manager.events import EventData, EventRegistry


def _event(candidate_id="a", event_type="fire", x=1.0, confidence=0.9):
    return EventData(candidate_id, "camera", event_type, confidence, x, 2.0, 0.0)


def test_event_registry_rejects_low_confidence_and_duplicates():
    registry = EventRegistry(0.7, 0.35)

    assert not registry.offer(_event(confidence=0.6))
    assert registry.offer(_event())
    assert not registry.offer(_event())
    assert not registry.offer(_event(candidate_id="b", x=1.2))
    assert registry.offer(_event(candidate_id="c", x=1.5))


def test_report_confirmation_is_explicit():
    registry = EventRegistry(0.7, 0.35)
    registry.offer(_event())

    attempted = registry.mark_attempt()
    assert attempted.attempts == 1
    assert registry.reported_count == 0

    reported = registry.mark_reported()
    assert reported.candidate_id == "a"
    assert registry.reported_count == 1
    assert registry.pending_count == 0
