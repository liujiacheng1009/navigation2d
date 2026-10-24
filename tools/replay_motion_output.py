"""Capture the planar motion port without changing the frozen replay fixtures."""
import re


def adapt_motion_output(source):
    pattern = r'motion_output_\s*->\s*Send\s*\(([^;]+)\);'
    source, count = re.subn(pattern, r'ReplaySendMotion(\1);', source)
    if re.search(r'motion_output_\s*->', source):
        raise ValueError('motion output adaptation incomplete')
    if not count:
        return source, 0
    marker = ' public:'
    begin = source.index(marker, source.index('class AutonomousExplorer')) + len(marker)
    capture = """
  void ReplaySendMotion(navigation2d::Twist2d value) {
    geometry_msgs::msg::Twist message;
    message.linear.x = value.linear;
    message.angular.z = value.angular;
    replay_.ProbeCommand(message);
  }
"""
    return source[:begin] + capture + source[begin:], count
