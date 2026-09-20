"""The configuration guard must fail loudly rather than guess a path."""
from __future__ import annotations

import pytest

from shensi_robot import config


def test_explicit_root_wins(tmp_path, monkeypatch):
    monkeypatch.setenv(config.ENV_VAR, str(tmp_path / "from-env"))
    assert config.resolve_config_root(tmp_path) == tmp_path


def test_env_var_is_used_when_no_argument(monkeypatch, tmp_path):
    monkeypatch.setenv(config.ENV_VAR, str(tmp_path))
    assert config.resolve_config_root(None) == tmp_path


def test_missing_everything_raises_with_instructions(monkeypatch):
    monkeypatch.delenv(config.ENV_VAR, raising=False)
    with pytest.raises(config.ConfigNotFoundError) as excinfo:
        config.resolve_config_root(None)
    message = str(excinfo.value)
    assert config.ENV_VAR in message
    assert "vendor/sdk" in message


def test_nonexistent_root_raises(monkeypatch):
    monkeypatch.delenv(config.ENV_VAR, raising=False)
    with pytest.raises(config.ConfigNotFoundError):
        config.resolve_config_root("/definitely/not/here")


def test_unknown_model_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="unknown model"):
        config.model_dir(tmp_path, "not-a-model")


def test_model_dir_reports_what_is_available(tmp_path):
    (tmp_path / "juxie_73").mkdir()
    assert config.model_dir(tmp_path, "juxie_73") == tmp_path / "juxie_73"
    with pytest.raises(config.ConfigNotFoundError, match="juxie_73"):
        config.model_dir(tmp_path, "juxie_62")
