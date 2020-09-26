class TransformException(Exception):
    pass


class ConnectivityException(TransformException):
    pass


class LookupException(TransformException):
    pass


class ExtrapolationException(TransformException):
    pass


class InvalidArgumentException(TransformException):
    pass


class TimeoutException(TransformException):
    pass
