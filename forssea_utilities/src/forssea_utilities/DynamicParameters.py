from rospy.impl.registration import get_service_manager


def shutdown_service_from_exception(exception):
    resolved_name = exception.message[exception.message.find("[") + 1:exception.message.find("]")]
    service = get_service_manager().get_service(resolved_name)
    service.shutdown()
