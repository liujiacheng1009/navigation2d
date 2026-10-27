"""Capture the planar motion port without changing the frozen replay fixtures."""
import re


def adapt_motion_output(source):
    pattern = r'motion_output_\s*->\s*Send\s*\(([^;]+)\);'
    source, count = re.subn(pattern, r'ReplaySendMotion(\1);', source)
    source, bindings = re.subn(r'\*\s*motion_output_\b', 'replay_motion_sink_', source)
    if re.search(r'motion_output_\s*(?:->|\.)', source):
        raise ValueError('motion output adaptation incomplete')
    if not count and not bindings:
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
    if bindings:
        capture += """
  struct ReplayMotionSink final : navigation2d::exploration::MotionOutput {
    explicit ReplayMotionSink(AutonomousExplorer& value) : owner(value) {}
    void Send(navigation2d::Twist2d value) override { owner.ReplaySendMotion(value); }
    AutonomousExplorer& owner;
  };
  ReplayMotionSink replay_motion_sink_{*this};
"""
    return source[:begin] + capture + source[begin:], count + bindings


def without_unused_motion_object(command, source, logic_only):
    """Logic startup removes this factory; do not load its unused ROS types."""
    if not logic_only:
        return list(command)
    unused = []
    for factory, filename in [('MakeRosMotionOutput', 'motion_output.cc.o'),
                              ('MakeExplorerSubscriptions', 'explorer_subscriptions.cc.o')]:
        if re.search(r'\b' + factory + r'\s*\(', source):
            raise ValueError('logic replay still constructs an endpoint: ' + factory)
        objects = [arg for arg in command if arg.endswith('/' + filename)]
        if len(objects) > 1:
            raise ValueError('ambiguous endpoint link inputs: ' + filename)
        unused.extend(objects)
    return [arg for arg in command if arg not in unused]
