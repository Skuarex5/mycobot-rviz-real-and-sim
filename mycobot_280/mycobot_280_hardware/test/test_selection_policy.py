import pytest
from mycobot_280_hardware.selection_policy import SelectionPolicy, StateSource


def test_busy_or_live_blocks_selection_and_jobs_pin_destination():
    p = SelectionPolicy()
    destination, generation = p.begin_job()
    assert destination == "simulation"
    with pytest.raises(ValueError):
        p.select(True)
    p.end_job()
    p.live = True
    with pytest.raises(ValueError):
        p.select(True)
    p.live = False
    p.select(True)
    assert p.begin_job() == ("physical", generation + 1)


def test_plans_do_not_survive_selection_roundtrip_or_late_results():
    p = SelectionPolicy()
    p.record_plan("same-trajectory", p.generation)
    assert p.valid_plan("same-trajectory")
    old = p.generation
    p.select(True)
    assert not p.valid_plan("same-trajectory")
    p.select(False)
    p.record_plan("late-result", old)
    assert not p.valid_plan("late-result")
    assert not p.valid_plan("same-trajectory")


def test_selected_states_require_progress_and_finite_complete_values():
    s = StateSource(["a", "b"])
    assert not s.update(["a"], [0.0], 1.0, 1.0)
    assert s.update(["a", "b"], [0.0, 0.0], 1.0, 1.0)
    assert not s.update(["a", "b"], [0.0, 0.0], 1.0, 1.4)
    assert not s.fresh(1.6)
    assert s.update(["a", "b"], [0.0, 0.0], 2.0, 2.0)
    assert not s.update(["a", "b"], [float("nan"), 0.0], 3.0, 2.1)
    # Clock reset is accepted as a new sample, never as freshness of the old one.
    assert s.update(["a", "b"], [0.1, 0.2], 0.1, 3.0)
