import unittest

g_autoware_lanelet2_extension_python_utility_imported = False

try:
    import autoware_lanelet2_extension_python.utility.query as query
    import autoware_lanelet2_extension_python.utility.utilities as utilities

    g_autoware_lanelet2_extension_python_utility_imported = True
except Exception as e:
    print(e)


class ImportTest(unittest.TestCase):
    """
    This test is intended to check
    'initialization of _autoware_lanelet2_extension_python_boost_python_*** \
    raised unreported exception'
    when we import this library without `import lanelet2`.

    We are preventing this by doing
    ```
    import lanelet2  # noqa: F401 # isort: skip
    ```
    in our __init__.py
    """

    def test_imported(self) -> None:
        self.assertTrue(g_autoware_lanelet2_extension_python_utility_imported)


if __name__ == "__main__":
    unittest.main()
