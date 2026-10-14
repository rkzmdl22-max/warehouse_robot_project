from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    ENVIRONMENT_CHECK = auto()
    BOTTLE_DETECTION = auto()
    WAIT_FOR_STABLE = auto()
    APPROACH = auto()
    MOTION_PLAN = auto()
    GRASP = auto()
    LIFT_VERIFY = auto()
    PLACE = auto()
    VERIFY = auto()
    COMPLETE = auto()
    SAFE_STOP = auto()
    REVALIDATE = auto()
    HOLD = auto()


class Event(Enum):
    START = auto()
    ENVIRONMENT_OK = auto()
    BOTTLE_DETECTED = auto()
    BOTTLE_STABLE = auto()
    APPROACH_OK = auto()
    PLAN_OK = auto()
    GRASP_OK = auto()
    LIFT_STABLE = auto()
    PLACE_OK = auto()
    VERIFY_OK = auto()
    FAULT = auto()
    CAUSE_CLEARED = auto()
    REVALIDATION_OK = auto()
    REVALIDATION_FAILED = auto()


NORMAL_TRANSITIONS = {
    (State.IDLE, Event.START): State.ENVIRONMENT_CHECK,
    (State.ENVIRONMENT_CHECK, Event.ENVIRONMENT_OK): State.BOTTLE_DETECTION,
    (State.BOTTLE_DETECTION, Event.BOTTLE_DETECTED): State.WAIT_FOR_STABLE,
    (State.WAIT_FOR_STABLE, Event.BOTTLE_STABLE): State.APPROACH,
    (State.APPROACH, Event.APPROACH_OK): State.MOTION_PLAN,
    (State.MOTION_PLAN, Event.PLAN_OK): State.GRASP,
    (State.GRASP, Event.GRASP_OK): State.LIFT_VERIFY,
    (State.LIFT_VERIFY, Event.LIFT_STABLE): State.PLACE,
    (State.PLACE, Event.PLACE_OK): State.VERIFY,
    (State.VERIFY, Event.VERIFY_OK): State.COMPLETE,
    (State.SAFE_STOP, Event.CAUSE_CLEARED): State.REVALIDATE,
    (State.REVALIDATE, Event.REVALIDATION_OK): State.ENVIRONMENT_CHECK,
    (State.REVALIDATE, Event.REVALIDATION_FAILED): State.HOLD,
    (State.HOLD, Event.CAUSE_CLEARED): State.REVALIDATE,
}


class HandoverMachine:
    def __init__(self) -> None:
        self.state = State.IDLE
        self.resume_target = State.ENVIRONMENT_CHECK

    def dispatch(self, event: Event) -> State:
        if event is Event.FAULT and self.state not in {State.IDLE, State.COMPLETE}:
            self.resume_target = self.state
            self.state = State.SAFE_STOP
            return self.state

        next_state = NORMAL_TRANSITIONS.get((self.state, event))
        if next_state is None:
            raise ValueError(f"invalid transition: {self.state.name} + {event.name}")
        self.state = next_state
        return self.state

