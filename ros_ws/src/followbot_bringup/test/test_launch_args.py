"""launch 引数の真偽値解釈 (DEVLOG S-20)。"""

import pytest

from followbot_bringup.launch_args import parse_bool


@pytest.mark.parametrize("text", ["true", "True", "TRUE", " true ",
                                  "1", "yes", "on", "ON"])
def test_真として読める値(text):
    assert parse_bool(text, "make_video") is True


@pytest.mark.parametrize("text", ["false", "False", "0", "no", "off"])
def test_偽として読める値(text):
    assert parse_bool(text, "make_video") is False


@pytest.mark.parametrize("text", ["", "ture", "2", "y", "n", "none", "-1"])
def test_解釈できない値は例外(text):
    # 黙って False にしない。MAKE_VIDEO=1 が無言で無効化された事故の再発防止
    with pytest.raises(ValueError):
        parse_bool(text, "make_video")


def test_例外に引数名と使える値が出る():
    with pytest.raises(ValueError) as e:
        parse_bool("ture", "make_video")
    assert "make_video" in str(e.value)
    assert "ture" in str(e.value)
    assert "true" in str(e.value)
