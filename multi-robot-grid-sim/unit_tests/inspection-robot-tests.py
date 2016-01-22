import unittest
from robots import InspectionRobot as ir


class IRTest(unittest.TestCase):

    def test_init_object(self):
        robot = ir(1, 1)
        self.assertTrue(robot.pos_x == 1)
        self.assertTrue(robot.pos_y == 1)

    def test_move_leq_max_vel(self):
        robot = ir(0, 0, max_vel=2)
        robot.move(2, 1)
        self.assertEqual(robot.pos_x, 2)
        self.assertEqual(robot.pos_y, 1)

    def test_move_x_greater_max_vel(self):
        robot = ir(0, 0)
        robot.move(2, 1)
        self.assertEqual(robot.pos_x, 1)
        self.assertEqual(robot.pos_y, 1)

    def test_move_y_greater_max_vel(self):
        robot = ir(0, 0)
        robot.move(1, 2)
        self.assertEqual(robot.pos_x, 1)
        self.assertEqual(robot.pos_y, 1)

    def test_move_both_greater_max_vel(self):
        robot = ir(0, 0)
        robot.move(2, 2)
        self.assertEqual(robot.pos_x, 1)
        self.assertEqual(robot.pos_y, 1)

if __name__ == '__main__':
    unittest.main()
