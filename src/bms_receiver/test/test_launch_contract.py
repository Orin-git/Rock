from pathlib import Path


NODE_FILE = Path(__file__).resolve().parents[1] / "bms_receiver" / "node.py"
BRINGUP_LAUNCH = (
    Path(__file__).resolve().parents[2]
    / "xw_bringup"
    / "launch"
    / "robot.launch.py"
)


def test_bringup_uses_controller_bms_receiver():
    text = BRINGUP_LAUNCH.read_text(encoding="utf-8")

    assert 'package="bms_receiver"' in text or "package='bms_receiver'" in text
    assert "bms_receiver_node" in text
    assert "byte_order" in text
    assert "comm_ok_value" in text


def test_runtime_parameters_are_refreshed_without_service_restart():
    text = NODE_FILE.read_text(encoding="utf-8")

    assert text.count('self.get_parameter("byte_order").value') >= 2
    assert text.count('self.get_parameter("comm_ok_value").value') >= 2
