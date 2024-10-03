import itertools
from typing import Iterable, TypeVar

from .models.configuration_state import ConfigurationState
from .models.edge import Edge
from .models.face import Face
from .models.facet import Facet
from .models.vertex import Vertex


def enumerate_vertex_plane_primitive_contacts(
    moving_part_vertices: Iterable[Vertex],
    stationary_part_faces: Iterable[Face],
):
    "Page 46, Figure 13"
    primitive_contacts = []
    bound = []
    for vertex, face in itertools.product(moving_part_vertices, stationary_part_faces):
        if vertex.lies_on(face):
            c = primitive_contacts.append((vertex, face))
            for edge in face.edges:
                assert isinstance(edge, Edge)
                if edge.is_convex and vertex.lies_on(edge):
                    g = edge.other_face(face)
                    bound.append((vertex, g, c))


def enumerate_plane_vertex_primitive_contacts(
    moving_part_faces: Iterable[Face],
    stationary_part_vertices: Iterable[Vertex],
):
    "Page 47, Figure 14"
    primitive_contacts = []
    bound = []
    for face, vertex in itertools.product(moving_part_faces, stationary_part_vertices):
        if vertex.lies_on(face):
            c = primitive_contacts.append((face, vertex))
            for edge in face.edges:
                assert isinstance(edge, Edge)
                if edge.is_convex and vertex.lies_on(edge):
                    g = edge.other_face(face)
                    bound.append((g, vertex, c))


def enumerate_line_line_primitive_contacts(
    moving_part_edges: Iterable[Edge],
    stationary_part_edges: Iterable[Edge],
):
    "Page 47, Figure 15"
    primitive_contacts = []
    bound = []
    for edge1, edge2 in itertools.product(moving_part_edges, stationary_part_edges):
        if edge1.intersects(edge2):
            c = primitive_contacts.append((edge1, edge2))
            for vertex in edge1.vertices:
                assert isinstance(vertex, Vertex)
                if vertex.is_convex and vertex.lies_on(edge2):
                    # TODO: Let F be the stationary part face adjacent to E2 whose normal is most nearly perpendicular to El
                    bound.append((vertex, f, c))
            for vertex in edge2.vertices:
                assert isinstance(vertex, Vertex)
                if vertex.is_convex and vertex.lies_on(edge1):
                    # TODO: Let F be the stationary part face adjacent to E1 whose normal is most nearly perpendicular to E2
                    bound.append((vertex, f, c))


def configuration_surface(X):
    pass


f = configuration_surface


# def calc_forbidden_zone():
#     """Page 49, equation 11"""
#     return {for X in R | f[i](X) < 0, f[i,j](X) < 0, f = 1 ...n[i]}
# def calc_forbidden_zone():
#     """Page 49, equation 13"""
#     return {for X in R | f[i](X) == 0, f[i,j](X) < 0, f = 1 ...n[i], X not in F }
def calc_forbidden_zone(i, R, f: dict, n) -> set:
    """Page 50, equation 14"""
    # return {for X in R | f[i](X) == 0, f[i,j](X) < 0, f = 1 ...n[i]}
    return {X for j in range(1, n[i]) for X in R if f[i](X) == 0 and f[i, j](X) < 0}


def enumerate_primitive_contacts(
    moving_part_vertices: Iterable[Vertex],
    stationary_part_faces: Iterable[Face],
    moving_part_faces: Iterable[Face],
    stationary_part_vertices: Iterable[Vertex],
    moving_part_edges: Iterable[Edge],
    stationary_part_edges: Iterable[Edge],
) -> "Iterable[tuple[Face, Vertex] | tuple[Vertex, Face] | tuple[Edge, Edge]]":
    """Page 39, figure 7"""
    primitive_contacts = set()
    for vertex in moving_part_vertices:
        for face in stationary_part_faces:
            if vertex.lies_on(face.surface):
                primitive_contacts.add((vertex, face))
    for face in moving_part_faces:
        for vertex in stationary_part_vertices:
            if vertex.lies_on(face.surface):
                primitive_contacts.add((face, vertex))
    for edge1 in moving_part_edges:
        for edge2 in stationary_part_edges:
            if edge1.intersects(edge2):
                primitive_contacts.add((edge1, edge2))
    return primitive_contacts


def generate_Contact_State_Adjacency_Graph():
    "Page 63, Figure 23"
    facets: dict[int, list[Facet]] = {}
    facet_primitive_contacts: dict = {}
    parents: dict[Facet, list[Facet]] = {}
    children: dict[Facet, list[Facet]] = {}
    configuration_states: dict[Facet, list[ConfigurationState]] = {}
    configuretion_states_pcs: dict[ConfigurationState, list[ConfigurationState]] = {}
    for i in range(4):
        facets[i + 1] = []
        for phi in facets[i]:
            children[phi] = []
            for primitive_contact_tuple in facet_primitive_contacts[phi]:
                if is_independent_in(primitive_contact_tuple, R_6):
                    phi_prime = Facet()
                    facet_primitive_contacts[phi_prime] = primitive_contact_tuple
                    support(phi_prime)
                    for phi_double_prime in facets[i + 1]:
                        if is_equal_facet(phi_prime, phi_double_prime):
                            facet_primitive_contacts[phi_prime] = merge(
                                facet_primitive_contacts[phi_prime],
                                facet_primitive_contacts[phi_double_prime],
                            )
                            parents[phi_double_prime].append(phi)
                            children[phi].append(phi_double_prime)
                        else:
                            facets[i + 1].append(phi_prime)
                            parents[phi_prime] = [phi]
                            children[phi].append(phi)

        for phi in facets[i + 1]:
            configuration_states[phi] = []
            for primitive_contact_tuple in facet_primitive_contacts[phi]:
                if is_feasible1(phi, *primitive_contact_tuple) and v_space(
                    *primitive_contact_tuple
                ) == v_space(support(phi)):
                    s = ConfigurationState()
                    configuretion_states_pcs[s] = primitive_contact_tuple
                    configuration_states[phi].append(s)
                    for phi_prime in parents[phi]:
                        for s_prime in configuration_states[phi_prime]:
                            if (
                                configuretion_states_pcs[s]
                                in configuretion_states_pcs[s_prime]
                            ):
                                parents[s].append(s_prime)
                                children[s_prime].append(s)


def grouping_C_states(facets:"dict[int,Iterable[Facet]]", configuration_states:"dict[Facet,Iterable[ConfigurationState]]") -> dict[ConfigurationState, Iterable[]]:
    "Page 64, Figure 24"
    sibs = dict()
    for i in range(4):
        for phi in facets[i]:
            for s in configuration_states[phi]:
                sibs[s] = []
        for s1, s2 in _pairs(configuration_states[phi]):
            if is_feasable2(phi, s1, s2):
                sibs[s1].append(s2)
                sibs[s2].append(s1)
T = TypeVar('T')
def _pairs(iter: Iterable[T]) -> "Iterable[tuple[T,T]]":
    "Return all pairs from a enumarable"
    return itertools.combinations(iter, 2)
