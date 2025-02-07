from functools import wraps

def custom_decorator(before=None, after=None):
    """
    创建一个自定义装饰器，可以在函数调用之前和之后执行自定义操作。
    
    :param before: 在函数调用之前执行的操作，接受同样的参数。
    :param after: 在函数调用之后执行的操作，接受同样的参数和返回值。
    :return: 一个装饰器。
    """
    def decorator(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if before:
                before(self, *args, **kwargs)
            result = func(self, *args, **kwargs)
            if after:
                after(self, result, *args, **kwargs)
            return result
        return wrapper
    return decorator

def apply_decorators(cls):
    for attr_name, attr_value in cls.__dict__.items():
        if callable(attr_value) and hasattr(attr_value, '_decorator'):
            decorated_func = attr_value._decorator(attr_value)
            setattr(cls, attr_name, decorated_func)
    return cls

