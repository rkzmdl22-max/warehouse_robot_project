import pytest

from handover_fsm.machine import Event, HandoverMachine, State


def test_happy_path_reaches_complete():
    machine = HandoverMachine()
    events = [
        Event.START,
        Event.ENVIRONMENT_OK,
        Event.BOTTLE_DETECTED,
        Event.BOTTLE_STABLE,
        Event.APPROACH_OK,
        Event.PLAN_OK,
        Event.GRASP_OK,
        Event.LIFT_STABLE,
        Event.PLACE_OK,
        Event.VERIFY_OK,
    ]
    for event in events:
        machine.dispatch(event)
    assert machine.state is State.COMPLETE


def test_fault_requires_revalidation():
    machine = HandoverMachine()
    machine.dispatch(Event.START)
    machine.dispatch(Event.ENVIRONMENT_OK)
    machine.dispatch(Event.FAULT)
    assert machine.state is State.SAFE_STOP
    machine.dispatch(Event.CAUSE_CLEARED)
    assert machine.state is State.REVALIDATE
    machine.dispatch(Event.REVALIDATION_OK)
    assert machine.state is State.ENVIRONMENT_CHECK


def test_invalid_transition_is_rejected():
    machine = HandoverMachine()
    with pytest.raises(ValueError):
        machine.dispatch(Event.GRASP_OK)
