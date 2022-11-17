from math import sin, cos, pi, ceil

class DrawableObject(object):
    def draw(self, at_step):
        print("To be overwritten - will draw a certain point in time:"), at_step

    def background_draw(self):
        print("Background draw.")

    @staticmethod
    def get_ellipse_points(center, main_axis_angle, radius1, radius2,
                           start_angle=0.0, end_angle=2 * pi):
        """Generate points of an ellipse, for drawing (y axis down)."""
        points = []
        ax = radius1 * cos(main_axis_angle)
        ay = radius1 * sin(main_axis_angle)
        bx = - radius2 * sin(main_axis_angle)
        by = radius2 * cos(main_axis_angle)
        N_full = 40  # Number of points on full ellipse.
        N = int(ceil((end_angle - start_angle) / (2 * pi) * N_full))
        N = max(N, 1)
        increment = (end_angle - start_angle) / N
        for i in range(N + 1):
            a = start_angle + i * increment
            c = cos(a)
            s = sin(a)
            x = c * ax + s * bx + center[0]
            y = - c * ay - s * by + center[1]
            points.append((x, y))
        return points

class Trajectory(DrawableObject):
    def __init__(self, points, canvas,
                 world_extents, canvas_extents,
                 standard_deviations = [],
                 point_size2 = 2,
                 background_color = "gray", cursor_color = "red",
                 position_stddev_color = "green", theta_stddev_color = "#ffc0c0"):
        self.points = points
        self.standard_deviations = standard_deviations
        self.canvas = canvas
        self.world_extents = world_extents
        self.canvas_extents = canvas_extents
        self.point_size2 = point_size2
        self.background_color = background_color
        self.cursor_color = cursor_color
        self.position_stddev_color = position_stddev_color
        self.theta_stddev_color = theta_stddev_color
        self.cursor_object = None
        self.cursor_object2 = None
        self.cursor_object3 = None
        self.cursor_object4 = None

    def background_draw(self):
        if len(self.points) > 0:
            p_xy_only = []
            for p in self.points:
                self.canvas.create_oval(\
                    p[0]-self.point_size2, p[1]-self.point_size2,
                    p[0]+self.point_size2, p[1]+self.point_size2,
                    fill=self.background_color, outline="")
                p_xy_only.append(p[0:2])
            if len(p_xy_only) >= 2:
                self.canvas.create_line(*p_xy_only, fill=self.background_color)

    def draw(self, at_step):
        if self.cursor_object:
            self.canvas.delete(self.cursor_object)
            self.cursor_object = None
            self.canvas.delete(self.cursor_object2)
            self.cursor_object2 = None
        if at_step < len(self.points):
            p = self.points[at_step]
            # Draw position (point).
            self.cursor_object = self.canvas.create_oval(\
                p[0]-self.point_size2-1, p[1]-self.point_size2-1,
                p[0]+self.point_size2+1, p[1]+self.point_size2+1,
                fill=self.cursor_color, outline="")
            # Draw error ellipse.
            if at_step < len(self.standard_deviations):
                stddev = self.standard_deviations[at_step]
                # Note this assumes correct aspect ratio.
                factor = self.canvas_extents[0] / self.world_extents[0]
                print("factor", factor)
                if len(stddev) == 2:
                    # main_axis_angle = 0
                    points = self.get_ellipse_points(p, 0, stddev[0] * factor, stddev[1] * factor)
                else:
                    points = self.get_ellipse_points(p, stddev[2], stddev[0] * factor, stddev[1] * factor)
                if self.cursor_object4:
                    self.canvas.delete(self.cursor_object4)
                self.cursor_object4 = self.canvas.create_line(
                    *points, fill=self.position_stddev_color)
            if len(p) > 2:
                # Draw heading standard deviation.
                if at_step < len(self.standard_deviations) and\
                   len(self.standard_deviations[0]) > 3:
                    angle = min(self.standard_deviations[at_step][3], pi)
                    points = self.get_ellipse_points(p, p[2], 30.0, 30.0,
                                                     -angle, angle)
                    points = [p[0:2]] + points + [p[0:2]]
                    if self.cursor_object3:
                        self.canvas.delete(self.cursor_object3)
                    self.cursor_object3 = self.canvas.create_polygon(
                        *points, fill=self.theta_stddev_color)
                # Draw heading.
                self.cursor_object2 = self.canvas.create_line(p[0], p[1],
                    p[0] + cos(p[2]) * 50,
                    p[1] - sin(p[2]) * 50,
                    fill = self.cursor_color)

class ScannerData(DrawableObject):
    def __init__(self, canvas, canvas_extents, scanner_range):
        self.canvas = canvas
        self.canvas_extents = canvas_extents
        self.cursor_object = None

        # Convert polar scanner measurements into xy form, in canvas coords.
        # Store the result in self.scan_polygons.
        self.scan_polygons = []

    def background_draw(self):
        # Draw x axis.
        self.canvas.create_line(
            self.canvas_extents[0]/2, self.canvas_extents[1]/2,
            self.canvas_extents[0]/2, 20,
            fill="black")
        self.canvas.create_text(
            self.canvas_extents[0]/2 + 10, 20, text="x" )
        # Draw y axis.
        self.canvas.create_line(
            self.canvas_extents[0]/2, self.canvas_extents[1]/2,
            20, self.canvas_extents[1]/2,
            fill="black")
        self.canvas.create_text(
            20, self.canvas_extents[1]/2 - 10, text="y" )

        camera_fov = 110
        self.canvas.create_arc(self.canvas_extents[0]/2-20, self.canvas_extents[1]/2-20,
            self.canvas_extents[0]/2+20, self.canvas_extents[1]/2+20,
                 start=(180 - camera_fov)/2,
                   extent=camera_fov,
                   fill="blue")

    def draw(self, at_step):
        if self.cursor_object:
            self.canvas.delete(self.cursor_object)
            self.cursor_object = None
        if at_step < len(self.scan_polygons):
            self.cursor_object = self.canvas.create_polygon(self.scan_polygons[at_step], fill="blue")

class Landmarks(DrawableObject):
    # In contrast other classes, Landmarks stores the original world coords and
    # transforms them when drawing.
    def __init__(self, landmarks, canvas, canvas_extents, world_extents, color = "gray"):
        self.landmarks = landmarks
        self.canvas = canvas
        self.canvas_extents = canvas_extents
        self.world_extents = world_extents
        self.color = color

    def background_draw(self):
        pass
        # for l in self.landmarks:
        #     if l[0] =='C':
        #         x, y = l[1:3]
        #         ll = to_world_canvas((x - l[3], y - l[3]), self.canvas_extents, self.world_extents)
        #         ur = to_world_canvas((x + l[3], y + l[3]), self.canvas_extents, self.world_extents)
        #         self.canvas.create_oval(ll[0], ll[1], ur[0], ur[1], fill=self.color)

    def draw(self, at_step):
        # Landmarks are background only.
        pass

class Points(DrawableObject):
    # Points, optionally with error ellipses.
    def __init__(self, points, canvas, color="red", ids=None, radius=5, ellipses=[], ellipse_factor=1.0):
        self.points = points
        self.ids = ids
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.ellipses = ellipses
        self.ellipse_factor = ellipse_factor
        self.cursor_objects = []

    def background_draw(self):
        pass

    def draw(self, at_step):
        if self.cursor_objects:
            for obj in self.cursor_objects: self.canvas.delete(obj)
            self.cursor_objects = []
        if at_step < len(self.points):
            for i in range(len(self.points[at_step])):
                # Draw point.
                c = self.points[at_step][i]
                self.cursor_objects.append(self.canvas.create_oval(
                    c[0] - self.radius, c[1] - self.radius,
                    c[0] + self.radius, c[1] + self.radius,
                    fill=self.color))
                if self.ids is not None:
                    self.cursor_objects.append(self.canvas.create_text(c[0]+25,c[1],fill="black",font="Helvetica 10", text=str(self.ids[at_step][i])))

                # Draw error ellipse if present.
                if at_step < len(self.ellipses) and i < len(self.ellipses[at_step]):
                    e = self.ellipses[at_step][i]
                    if len(e) == 2:
                        # set the main_axis_angle = 0
                        points = self.get_ellipse_points(c, 0, e[0] * self.ellipse_factor,
                                                        e[1] * self.ellipse_factor)
                    else:
                        points = self.get_ellipse_points(c, e[2], e[0] * self.ellipse_factor,
                                                        e[1] * self.ellipse_factor)
                    self.cursor_objects.append(self.canvas.create_line(
                        *points, fill=self.color))

# Particles are like points but add a direction vector.
class Particles(DrawableObject):
    def __init__(self, particles, canvas, color = "red", radius = 1.0,
                 vector = 8.0):
        self.particles = particles
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.vector = vector
        self.cursor_objects = []

    def background_draw(self):
        pass

    def draw(self, at_step):
        if self.cursor_objects:
            for obj in self.cursor_objects: self.canvas.delete(obj)
            self.cursor_objects = []
        if at_step < len(self.particles):
            for c in self.particles[at_step]:
                self.cursor_objects.append(self.canvas.create_oval(
                    c[0]-self.radius, c[1]-self.radius,
                    c[0]+self.radius, c[1]+self.radius,
                    fill=self.color, outline=self.color))
                self.cursor_objects.append(self.canvas.create_line(
                    c[0], c[1],
                    c[0] + cos(c[2]) * self.vector,
                    c[1] - sin(c[2]) * self.vector,
                    fill = self.color))
