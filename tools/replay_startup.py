"""Omit unused ROS communication setup in an explicitly scoped logic replay."""


def without_endpoints(source):
    forms = ['Node("sweepnav_autonomous_explorer")',
             'Node("sweepnav_autonomous_explorer", options)']
    matches = [form for form in forms if source.count(form)]
    if len(matches) != 1 or source.count(matches[0]) != 1:
        raise ValueError('unsupported node constructor for logic replay')
    constructor = matches[0]
    markers = ['    command_publisher_ =', '    motion_output_ =']
    found = [source.index(marker) for marker in markers if marker in source]
    if len(found) != 1:
        raise ValueError('unsupported motion setup for logic replay')
    begin = found[0]
    end = source.index('    PublishStatus("WAITING"', begin)
    setup = source[begin:end]
    if not all(name in setup for name in ['create_publisher', 'create_wall_timer']):
        raise ValueError('unsupported communication setup for logic replay')
    if 'create_subscription' not in setup and 'MakeExplorerSubscriptions(' not in setup:
        raise ValueError('unsupported subscription setup for logic replay')
    source = source[:begin] + source[end:]
    initial = 'options' if constructor == forms[1] else ''
    options = (f'rclcpp::NodeOptions({initial}).enable_rosout(false).start_parameter_services(false)'
               '.start_parameter_event_publisher(false)')
    return source.replace(constructor, f'Node("sweepnav_autonomous_explorer", {options})')
