"""Capture the borrowed debug-message port, retaining real resource lifetimes."""
import re


def adapt_debug_output(source):
    source, bindings = re.subn(r'\*\s*debug_output_\b', 'replay_debug_sink_', source)
    if re.search(r'debug_output_\s*(?:->|\.)', source):
        raise ValueError('debug output adaptation incomplete')
    if not bindings:
        return source, 0
    marker = ' public:'
    begin = source.index(marker, source.index('class AutonomousExplorer')) + len(marker)
    capture = '''
  struct ReplayDebugSink final : navigation2d::exploration_ros::DebugReportSink {
    explicit ReplayDebugSink(AutonomousExplorer& value) : owner(value) {}
    void SendPath(navigation2d::exploration_ros::DebugPathChannel,
                  const nav_msgs::msg::Path& message) override { owner.replay_.ProbePublish(message); }
    void SendMap(navigation2d::exploration_ros::DebugMapChannel,
                 const nav_msgs::msg::OccupancyGrid& message) override { owner.replay_.ProbePublish(message); }
    void SendCarrot(const geometry_msgs::msg::PoseStamped& message) override { owner.replay_.ProbePublish(message); }
    void SendFrontiers(const geometry_msgs::msg::PoseArray& message) override { owner.replay_.ProbePublish(message); }
    void SendFrontierDebug(const std_msgs::msg::String& message) override { owner.replay_.ProbePublish(message); }
    AutonomousExplorer& owner;
  };
  ReplayDebugSink replay_debug_sink_{*this};
'''
    return source[:begin] + capture + source[begin:], bindings
