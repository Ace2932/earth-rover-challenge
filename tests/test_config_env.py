"""Env overrides must mean what they say (issue #36).

Config.from_env cast each override with `type(current_value)(env)`. For a bool
that is wrong in the most dangerous direction: bool("0") is True, so a flag set to
0 in .env to turn a feature OFF would turn it ON.
"""
import pytest

from envcfg import coerce
from waypoint_follower import Config


@pytest.mark.parametrize("raw", ["0", "false", "False", "no", "off", "", "  0  "])
def test_falsey_strings_turn_a_flag_off(raw):
    assert coerce(True, raw) is False


@pytest.mark.parametrize("raw", ["1", "true", "True", "yes", "on"])
def test_truthy_strings_turn_a_flag_on(raw):
    assert coerce(False, raw) is True


def test_numbers_still_parse_as_numbers():
    assert coerce(1.5, "2.5") == 2.5
    assert coerce(3, "7") == 7
    assert isinstance(coerce(3, "7"), int)


def test_strings_pass_through():
    assert coerce("a", "b") == "b"


def test_a_malformed_number_says_which_value_was_bad():
    with pytest.raises(ValueError) as e:
        coerce(1.0, "not-a-number")
    assert "not-a-number" in str(e.value)


def test_config_from_env_uses_it(monkeypatch):
    monkeypatch.setenv("CRUISE", "0.42")
    assert Config.from_env().cruise == pytest.approx(0.42)


def test_config_from_env_names_the_variable_it_could_not_parse(monkeypatch):
    monkeypatch.setenv("CRUISE", "fast")
    with pytest.raises(ValueError) as e:
        Config.from_env()
    assert "CRUISE" in str(e.value)
