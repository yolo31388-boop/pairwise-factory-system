"""Pytest fixtures/hooks for the factory test suite.

The test classes use unittest-style assertions (e.g. assertIsNotNone)
without inheriting from unittest.TestCase. Mix the standard assertion
helpers into any plain test class at collection time so the tests run
as written.
"""

import unittest


def pytest_collection_modifyitems(session, config, items):
    for item in items:
        cls = getattr(item, "cls", None)
        if cls is None or issubclass(cls, unittest.TestCase):
            continue
        for name in dir(unittest.TestCase):
            if name.startswith("assert") and not hasattr(cls, name):
                setattr(cls, name, getattr(unittest.TestCase, name))
