from unittest import TestCase

from ictsm.compact_location import compact_location, expand_location, ROW_FACTOR

param_list = [(0, 0), (0, 1), (1, 0), (1, 1), (ROW_FACTOR - 1, 0)]


class TestCompactLocationReversibility(TestCase):
    def test_works_as_expected(self):
        for (x, y) in param_list:
            with self.subTest():
                self.assertEqual(expand_location(compact_location(x, y)), (x, y))
