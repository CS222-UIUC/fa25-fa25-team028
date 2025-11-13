import numpy as np

# PHYSICS HELPING FUNCTIONS

def polygon_area_and_centroid(verts_ccw):
    """
    Calculating area and center of mass
    """
    v = np.asarray(verts_ccw, dtype=float)
    x = v[:, 0]; y = v[:, 1]
    x1 = np.roll(x, -1); y1 = np.roll(y, -1)
    cross = x * y1 - x1 * y
    A2 = cross.sum()  # 2 * area
    area = 0.5 * A2
    if abs(A2) < 1e-12:
        raise ValueError("Area value error")
    cx = ( (x + x1) * cross ).sum() / (3.0 * A2)
    cy = ( (y + y1) * cross ).sum() / (3.0 * A2)
    return abs(area), np.array([cx, cy])


# def polygon_inertia_about_centroid(mass, verts_ccw):
#     """
#     Inertia about centroid
#     """
#     v = np.asarray(verts_ccw, dtype=float)
#     x = v[:, 0]; y = v[:, 1]
#     x1 = np.roll(x, -1); y1 = np.roll(y, -1)
#     cross = x * y1 - x1 * y
#     denom = 12.0 * polygon_area_and_centroid(v)[0]
#     I = ( (x*x + x*x1 + x1*x1) + (y*y + y*y1 + y1*y1) ) * cross
#     I = mass * I.sum() / denom
#     return float(abs(I))

def polygon_inertia_about_centroid(mass, verts_ccw):
    """
    Inertia of a polygon about its centroid.
    verts_ccw: (N,2) array, counter clockwise vertices
    """
    v = np.asarray(verts_ccw, dtype=float)

    area, centroid = polygon_area_and_centroid(v)
    if area <= 0.0:
        raise ValueError("Polygon area must be positive")
    area = abs(area)

    v = v - centroid

    x = v[:, 0]; y = v[:, 1]
    x1 = np.roll(x, -1); y1 = np.roll(y, -1)

    cross = x * y1 - x1 * y
    denom = 12.0 * area          # density = mass / area

    I_terms = ((x*x + x*x1 + x1*x1) +
               (y*y + y*y1 + y1*y1)) * cross

    I = mass * I_terms.sum() / denom
    return float(abs(I))


def rotation_matrix(theta):
    c = np.cos(theta); s = np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)


def cross2D(a, b):
    """2D vector cross product"""
    return a[0]*b[1] - a[1]*b[0]


# POLYGONSHAPE CLASS

class PolygonShape:
    def __init__(self, local_vertices_ccw):
        """
        This class does not provide mass attribute! Only 
        calculates shape
        """
        v = np.array(local_vertices_ccw, dtype=float)
        if v.shape[0] < 3:
            raise ValueError("Polygon requires at least 3 vertices")
        self.local_vertices = v
        area, centroid = polygon_area_and_centroid(v)
        self.area = area
        self.local_centroid = centroid  

    @property
    def local_edges(self):
        v = self.local_vertices
        return np.roll(v, -1, axis=0) - v

    @property
    def local_normals(self):
        """
        Normal lines of every edge
        """
        e = self.local_edges
        n = np.stack([-e[:,1], e[:,0]], axis=1)  # rotate by 90 degrees -> perpendicular
        lens = np.linalg.norm(n, axis=1, keepdims=True)
        n = n / np.clip(lens, 1e-12, None)
        return n


# RIGID BODY CLASS

class RigidBody2D:
    def __init__(self, shape: PolygonShape, mass=None, density=None,
                 position=(0,0), angle=0.0, velocity=(0,0), angular_velocity=0.0):
        """
        mass is given by given mass or calculated using density
        """
        self.shape = shape
        self.position = np.array(position, dtype=float)  # center of mass
        self.angle = float(angle)
        self.velocity = np.array(velocity, dtype=float)
        self.angular_velocity = float(angular_velocity)

        # mass
        if (mass is None) == (density is None):
            raise ValueError("have to choose between mass or density")
        self.mass = float(mass) if mass is not None else float(density) * shape.area
        self.inv_mass = 0.0 if self.mass == 0 else 1.0 / self.mass

        I_c = polygon_inertia_about_centroid(self.mass, shape.local_vertices)
        self.inertia = float(I_c)          
        self.inv_inertia = 0.0 if self.inertia == 0 else 1.0 / self.inertia

        self.force = np.zeros(2, dtype=float)
        self.torque = 0.0

        # position is center of mass
        self._R = rotation_matrix(self.angle)

    # POSITION AND POSTURE

    def _update_rotation(self):
        self._R = rotation_matrix(self.angle)

    def world_vertices(self):
        """
        vertices of the polygon in the world
        """
        v_local = self.shape.local_vertices - self.shape.local_centroid
        return (v_local @ self._R.T) + self.position

    def world_normals(self):
        """normals that are only affected by rotation"""
        n_local = self.shape.local_normals
        return n_local @ self._R.T

    def aabb(self):
        """Axis-aligned bounding box"""
        wv = self.world_vertices()
        return wv.min(axis=0), wv.max(axis=0)

    # FORCE AND INTEGRATION

    def clear_forces(self):
        self.force[:] = 0.0
        self.torque = 0.0

    def apply_force(self, force, world_point=None):
        """
        applying force on a point
        """
        f = np.array(force, dtype=float)
        self.force += f
        if world_point is not None:
            r = np.array(world_point, dtype=float) - self.position  # from the point of interaction to center of mass 
            self.torque += cross2D(r, f)  

    def apply_impulse(self, impulse, world_point=None):
        """
        Impulse -> change in momentum
        """
        j = np.array(impulse, dtype=float)
        self.velocity += self.inv_mass * j
        if world_point is not None:
            r = np.array(world_point, dtype=float) - self.position
            self.angular_velocity += self.inv_inertia * cross2D(r, j)

    def integrate(self, dt):
        """
        half - implicit Euler Integration:
        v += (F/m)*dt, ω += (τ/I)*dt
        x += v*dt, θ += ω*dt
        """
        # linear acceleration & angular acceleration
        a = self.force * self.inv_mass
        alpha = self.torque * self.inv_inertia

        # update velocity
        self.velocity += a * dt
        self.angular_velocity += alpha * dt

        # update position
        self.position += self.velocity * dt
        self.angle += self.angular_velocity * dt
        self._update_rotation()

        # clearing all forces after the interaction!!!!
        self.clear_forces()

    # ---------- AI - Generated Helper Functions, Not Utilized  ----------

    def support_point(self, direction_world):
        """
        Support function: returns the vertex (in world coordinates) 
        with the maximum projection along the given direction.
        Useful for algorithms like GJK, EPA, or SAT.
        """
        d = np.array(direction_world, dtype=float)
        wv = self.world_vertices()
        idx = np.argmax(wv @ d)
        return wv[idx]

    def project_onto_axis(self, axis_world):
        """
        SAT projection: returns the scalar [min, max] range of the polygon 
        when projected onto the given world-space axis.
        """
        a = np.array(axis_world, dtype=float)
        a = a / (np.linalg.norm(a) + 1e-12)
        wv = self.world_vertices()
        proj = wv @ a
        return proj.min(), proj.max()