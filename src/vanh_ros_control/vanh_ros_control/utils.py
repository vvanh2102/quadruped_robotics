#!/usr/bin/python3
from threading import Thread
from typing import Callable

def runInThread(func: Callable):
    """
    Decorator make function run in another thread
    """
    def inner(*args, **kwargs):
        Thread(target=func, args=args, kwargs=kwargs, daemon=True).start()
    return inner