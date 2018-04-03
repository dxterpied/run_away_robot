# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ---# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
from robot import *
from math import *
from matrix import *
import random
from KF import *


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if OTHER is not None:
        prev_position_pred = OTHER[5]
        error = distance_between(target_measurement, prev_position_pred)
        if error < 0.015:
            max_pred = 10
        else:
            max_pred = 1
    else:
        max_pred = 1
    pred_robot, OTHER = target_position_pred(target_measurement, OTHER)
    caught_distance = distance_between(pred_robot.sense(), hunter_position)
    i = 1
    while caught_distance > max_distance * i and i < max_pred:
        pred_robot.move_in_circle()
        caught_distance = distance_between(
            pred_robot.sense(), hunter_position)
        i += 1

    px, py = pred_robot.sense()

    turning = angle_trunc(
        atan2(py - hunter_position[1], px - hunter_position[0]) - hunter_heading)
    distance = min(caught_distance, max_distance)
    return turning, distance, OTHER


def target_position_pred(target_measurement, OTHER=None):
    if OTHER is None:
        target_xy_pred = target_measurement
        number_measurement = 0
        heading = 0
        t_list = [0]
        d_list = [0]
        pred_robot = robot(0, 0, 0, 0, 0)
    else:
        prev_number_measurement = OTHER[0]
        prev_measurement = OTHER[1]
        prev_heading = OTHER[2]
        t_list = OTHER[3]
        d_list = OTHER[4]

        number_measurement = prev_number_measurement + 1
        d = distance_between(target_measurement, prev_measurement)
        d_list.append(d)
        total_d = sum(d_list)
        average_d = total_d / number_measurement

        heading = atan2(target_measurement[1] - prev_measurement[1],
                        target_measurement[0] - prev_measurement[0])
        t = heading - prev_heading
        t = angle_trunc(t)
        t_list.append(t)
        total_t = sum(t_list)
        average_t = total_t / number_measurement

        tmp_d = tmp_t = 0
        for i in range(number_measurement):
            tmp_d += pow(d_list[i] - average_d, 2)
            tmp_t += pow(t_list[i] - average_t, 2) 
        delta_d = tmp_d / number_measurement
        delta_t = tmp_t / number_measurement

        print delta_d, average_d, delta_t, average_t

        pred_robot = robot(
            target_measurement[0], target_measurement[1], heading, average_t, average_d)
        pred_robot.move_in_circle()
        target_xy_pred = pred_robot.sense()
    OTHER = [number_measurement, target_measurement,
             heading, t_list, d_list, target_xy_pred]
    return pred_robot, OTHER


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * \
        target_bot.distance  # 0.97 is an example. It will change.
    # hunter must be within 0.02 step size to catch target
    separation_tolerance = 0.02 * target_bot.distance
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 10000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(
            hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 10000:
            print "It took too many steps to catch the target."
    return caught


# def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
#     """Returns True if your next_move_fcn successfully guides the hunter_bot
#     to the target_bot. This function is here to help you understand how we 
#     will grade your submission."""
#     max_distance = 0.97 * \
#         target_bot.distance  # 0.98 is an example. It will change.
#     # hunter must be within 0.02 step size to catch target
#     separation_tolerance = 0.02 * target_bot.distance
#     caught = False
#     ctr = 0
#     # For Visualization
#     import turtle
#     window = turtle.Screen()
#     window.bgcolor('white')
#     chaser_robot = turtle.Turtle()
#     chaser_robot.shape('arrow')
#     chaser_robot.color('blue')
#     chaser_robot.resizemode('user')
#     chaser_robot.shapesize(0.3, 0.3, 0.3)
#     broken_robot = turtle.Turtle()
#     broken_robot.shape('turtle')
#     broken_robot.color('green')
#     broken_robot.resizemode('user')
#     broken_robot.shapesize(0.3, 0.3, 0.3)
#     size_multiplier = 15.0  # change size of animation
#     chaser_robot.hideturtle()
#     chaser_robot.penup()
#     chaser_robot.goto(hunter_bot.x*size_multiplier,
#                       hunter_bot.y*size_multiplier-100)
#     chaser_robot.showturtle()
#     broken_robot.hideturtle()
#     broken_robot.penup()
#     broken_robot.goto(target_bot.x*size_multiplier,
#                       target_bot.y*size_multiplier-100)
#     broken_robot.showturtle()
#     measuredbroken_robot = turtle.Turtle()
#     measuredbroken_robot.shape('circle')
#     measuredbroken_robot.color('red')
#     measuredbroken_robot.penup()
#     measuredbroken_robot.resizemode('user')
#     measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
#     broken_robot.pendown()
#     chaser_robot.pendown()
#     # End of Visualization
#     # We will use your next_move_fcn until we catch the target or time expires.
#     while not caught and ctr < 1000:
#         # Check to see if the hunter has caught the target.
#         hunter_position = (hunter_bot.x, hunter_bot.y)
#         target_position = (target_bot.x, target_bot.y)
#         separation = distance_between(hunter_position, target_position)
#         if separation < separation_tolerance:
#             print "You got it right! It took you ", ctr, " steps to catch the target."
#             caught = True

#         # The target broadcasts its noisy measurement
#         target_measurement = target_bot.sense()

#         # This is where YOUR function will be called.
#         turning, distance, OTHER = next_move_fcn(
#             hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

#         # Don't try to move faster than allowed!
#         if distance > max_distance:
#             distance = max_distance

#         # We move the hunter according to your instructions
#         hunter_bot.move(turning, distance)

#         # The target continues its (nearly) circular motion.
#         target_bot.move_in_circle()
#         # Visualize it
#         measuredbroken_robot.setheading(target_bot.heading*180/pi)
#         measuredbroken_robot.goto(
#             target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
#         measuredbroken_robot.stamp()
#         broken_robot.setheading(target_bot.heading*180/pi)
#         broken_robot.goto(target_bot.x*size_multiplier,
#                           target_bot.y*size_multiplier-100)
#         chaser_robot.setheading(hunter_bot.heading*180/pi)
#         chaser_robot.goto(hunter_bot.x*size_multiplier,
#                           hunter_bot.y*size_multiplier-100)
#         # End of visualization
#         ctr += 1
#         if ctr >= 1000:
#             print "It took too many steps to catch the target."
#     return caught


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        # now I can keep track of history
        OTHER = (measurements, hunter_positions, hunter_headings)
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        # now I can always refer to these variables
        measurements, hunter_positions, hunter_headings = OTHER

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target
    distance = max_distance  # full speed ahead!
    return turning, distance, OTHER


target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2.*target.distance  # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, next_move)
