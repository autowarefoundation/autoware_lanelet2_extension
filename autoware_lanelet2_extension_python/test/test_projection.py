import unittest

from autoware_lanelet2_extension_python.projection import MGRSProjector


class MGRSProjectorTest(unittest.TestCase):
    def test_mgrs_initialization(self) -> None:
        m = MGRSProjector()
        self.assertFalse(m.isMGRSCodeSet())

    def test_mgrs_set(self) -> None:
        m = MGRSProjector()
        m.setMGRSCode("53SQA")
        self.assertEqual(m.getProjectedMGRSGrid(), "53SQA")
        # now set
        self.assertTrue(m.isMGRSCodeSet())


if __name__ == "__main__":
    unittest.main()
