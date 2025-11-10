import unittest

from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2


class MGRSProjectorTest(unittest.TestCase):
    def test_mgrs_initialization(self) -> None:
        m = MGRSProjector(lanelet2.io.Origin(0, 0))
        self.assertFalse(m.isMGRSCodeSet())

    def test_mgrs_set(self) -> None:
        m = MGRSProjector()
        m.setMGRSCode("53SQA")
        # now set
        self.assertTrue(m.isMGRSCodeSet())


if __name__ == "__main__":
    unittest.main()
