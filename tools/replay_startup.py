"""Omit unused ROS communication setup in an explicitly scoped logic replay."""


def without_endpoints(source):
    constructor = 'Node("sweepnav_autonomous_explorer")'
    if source.count(constructor) != 1:
        raise ValueError('unsupported node constructor for logic replay')
    markers = ['    command_publisher_ =', '    motion_output_ =']
    found = [source.index(marker) for marker in markers if marker in source]
    if len(found) != 1:
        raise ValueError('unsupported motion setup for logic replay')
    begin = found[0]
    end = source.index('    PublishStatus("WAITING"', begin)
    setup = source[begin:end]
    if not all(name in setup for name in ['create_publisher', 'create_subscription', 'create_wall_timer']):
        raise ValueError('unsupported communication setup for logic replay')
    source = source[:begin] + source[end:]
    options = ('rclcpp::NodeOptions().enable_rosout(false).start_parameter_services(false)'
               '.start_parameter_event_publisher(false)')
    return source.replace(constructor, f'Node("sweepnav_autonomous_explorer", {options})')
