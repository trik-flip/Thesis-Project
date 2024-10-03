import itertools
from enum import Enum, auto
from functools import singledispatch
from typing import Iterable, TypeVar, cast, Union, Tuple

from pipe import chain, dedup, groupby, select, traverse, where, sort, take

from ..model.configuration_state import ConfigurationState
from ..model.edge import Edge
from ..model.face import Face
from ..model.facet import Facet
from ..model.vertex import Vertex


pc = Union[Tuple[Face, Vertex], Tuple[Vertex, Face], Tuple[Edge, Edge]]
R_6 = None


class PrimitiveContactsType(Enum):
    VERTEX_PLANE = auto()
    PLANE_VERTEX = auto()
    LINE_LINE = auto()


@singledispatch
def enumerate_primitive_contact(moving: Iterable, stationary: Iterable):
    if all((isinstance(v, Vertex) for v in moving)) and all(
        (isinstance(f, Face) for f in stationary)
    ):
        moving = cast(Iterable[Vertex], moving)
        stationary = cast(Iterable[Face], stationary)
        return _enumerate_vertex_plane_primitive_contacts(moving, stationary)
    elif all((isinstance(f, Face) for f in moving)) and all(
        (isinstance(v, Vertex) for v in stationary)
    ):
        moving = cast(Iterable[Face], moving)
        stationary = cast(Iterable[Vertex], stationary)
        return _enumerate_plane_vertex_primitive_contacts(moving, stationary)
    elif all((isinstance(e, Edge) for e in moving)) and all(
        (isinstance(e, Edge) for e in stationary)
    ):
        moving = cast(Iterable[Edge], moving)
        stationary = cast(Iterable[Edge], stationary)
        return _enumerate_line_line_primitive_contacts(moving, stationary)
    else:
        raise Exception("This code is unreachable")


@enumerate_primitive_contact.register
def enumerate_primitive_contact(moving: Iterable[Vertex],
                                stationary: Iterable[Face]):
    """enumerate over Vertex on Face contacs
    where the Vertex contacts are able to move"""
    ...


@enumerate_primitive_contact.register
def enumerate_primitive_contact(moving: Iterable[Face],
                                stationary: Iterable[Vertex]):
    """enumerate over Face on Vertex contacs
    where the Face contacts are able to move"""
    ...


@enumerate_primitive_contact.register
def enumerate_primitive_contact(moving: Iterable[Edge],
                                stationary: Iterable[Edge]):
    """enumerate over Edge on Edge contacs
    where the Edge contacts are able to move"""
    ...


def _enumerate_vertex_plane_primitive_contacts(
    moving_part_vertices: Iterable[Vertex],
    stationary_part_faces: Iterable[Face],
):
    "Page 46, Figure 13"
    primitive_contacts = []
    bound = []
    pairs = itertools.product(moving_part_vertices,
                              stationary_part_faces)
    for vertex, face in pairs | where(lambda v, f: v.lies_on(f)):
        primitive_contacts.append(c := (vertex, face))
        for edge in face.edges | where(lambda e: e.is_convex
                                       and vertex.lies_on(e)):
            assert isinstance(edge, Edge)
            g = edge.other_face(face)
            bound.append((vertex, g, c))
    return bound


def _enumerate_plane_vertex_primitive_contacts(
    moving_part_faces: Iterable[Face],
    stationary_part_vertices: Iterable[Vertex],
):
    "Page 47, Figure 14"
    primitive_contacts: list[tuple[Face, Vertex]] = []
    bound: list[tuple[Face, Vertex, tuple[Face, Vertex]]] = []

    pairs = itertools.product(moving_part_faces,
                              stationary_part_vertices)
    for face, vertex in pairs | where(lambda f, v: v.lies_on(f)):
        primitive_contacts.append(c := (face, vertex))
        for edge in face.edges | where(lambda e: e.is_convex
                                       and vertex.lies_on(e)):
            g = edge.other_face(face)
            bound.append((g, vertex, c))
    return bound


def _enumerate_line_line_primitive_contacts(
    moving_part_edges: Iterable[Edge],
    stationary_part_edges: Iterable[Edge],
) -> "Iterable[tuple[Vertex, tuple[Edge,Edge]]]":
    "Page 47, Figure 15"
    primitive_contacts: list[tuple[Edge, Edge]] = []
    bound: list[Vertex, Face, tuple[Edge, Edge]] = []

    pairs = itertools.product(moving_part_edges,
                              stationary_part_edges)
    for edge1, edge2 in pairs | where(lambda e1, e2: e1.intersects(e2)):
        primitive_contacts.append(c := (edge1, edge2))
        for vertex in edge1.vertices | where(lambda vertex: vertex.is_convex
                                             and vertex.lies_on(edge2)):
            f = list(
                stationary_part_edges
                | where(lambda x: x.is_adjacent_to(edge2))
                | sort(lambda x: x.normal.
                       perpendicularity_with_respect_to(edge1))
                | take(1)
            )[0]
            # TODO: Let F be the stationary part face adjacent
            # to E2 whose normal is most nearly perpendicular to El
            bound.append((vertex, f, c))
        for vertex in edge2.vertices | where(lambda vertex: vertex.is_convex
                                             and vertex.lies_on(edge1)):
            f = list(
                stationary_part_edges
                | where(lambda x: x.is_adjacent_to(edge1))
                | sort(lambda x: x.normal.
                       perpendicularity_with_respect_to(edge2))
                | take(1)
            )[0]
            # TODO: Let F be the stationary part face adjacent
            # to E1 whose normal is most nearly perpendicular to E2
            bound.append((vertex, f, c))
    return bound


def configuration_surface(X):
    pass


f = configuration_surface


# def calc_forbidden_zone():
#     """Page 49, equation 11"""
#     return {for X in R | f[i](X) < 0, f[i,j](X) < 0, f = 1 ...n[i]}
# def calc_forbidden_zone():
#     """Page 49, equation 13"""
#     return {for X in R | f[i](X) == 0, f[i,j](X) < 0, f = 1 ...n[i],
#        X not in F }
def calc_forbidden_zone(i, R, f: dict, n) -> set:
    """Page 50, equation 14"""
    # return {for X in R | f[i](X) == 0, f[i,j](X) < 0, f = 1 ...n[i]}
    return {
        X for j in range(1, n[i]) for X in R
        if f[i](X) == 0 and f[i, j](X) < 0
    }


def enumerate_primitive_contacts(
    moving_v: Iterable[Vertex],
    stationary_f: Iterable[Face],
    moving_f: Iterable[Face],
    stationary_v: Iterable[Vertex],
    moving_e: Iterable[Edge],
    stationary_e: Iterable[Edge],
) -> "Iterable[tuple[Face, Vertex] | tuple[Vertex, Face] | tuple[Edge, Edge]]":
    """Page 39, figure 7"""
    pcs: set[pc] = set()
    for v, f in itertools.product(moving_v, stationary_f)
    | where(lambda v, f: v.lies_on(f.surface)):
        pcs.add((v, f))
    for f, v in itertools.product(moving_f, stationary_v)
    | where(lambda f, v: v.lies_on(f.surface)):
        pcs.add((f, v))
    for edge1, edge2 in itertools.product(moving_e, stationary_e)
    | where(lambda edge1, edge2: edge1.intersects(edge2)):
        pcs.add((edge1, edge2))
    return pcs


def generate_Contact_State_Adjacency_Graph():
    "Page 63, Figure 23"
    facets: dict[int, list[Facet]] = {}
    facet_pcs: dict[Facet, ] = {}
    parents: dict[Facet, list[Facet]] = {}
    children: dict[Facet, list[Facet]] = {}
    css: dict[Facet, list[ConfigurationState]] = {}
    css_pcs: dict[ConfigurationState,
                  list[ConfigurationState]] = {}
    for i in range(4):
        facets[i + 1] = []
        for phi in facets[i]:
            children[phi] = []
            for pc_tuple in facet_pcs[phi]
            | where(lambda pcs: is_independent_in(pcs, R_6)):
                phi_prime = Facet()
                facet_pcs[phi_prime] = pc_tuple
                support(phi_prime)
                for phi_double_prime in facets[i + 1]:
                    if is_equal_facet(phi_prime, phi_double_prime):
                        facet_pcs[phi_prime] = merge(
                            facet_pcs[phi_prime],
                            facet_pcs[phi_double_prime],
                        )
                        parents[phi_double_prime].append(phi)
                        children[phi].append(phi_double_prime)
                    else:
                        facets[i + 1].append(phi_prime)
                        parents[phi_prime] = [phi]
                        children[phi].append(phi)

        for phi in facets[i + 1]:
            css[phi] = []
            for pc_tuple in facet_pcs[phi]
            | where(lambda pcs:
                    is_feasible1(phi, *pcs) and
                    v_space(*pcs) == v_space(support(phi))):
                s = ConfigurationState()
                css_pcs[s] = pc_tuple
                css[phi].append(s)
                for phi_prime, s_prime in itertools.product(
                    parents[phi], css[phi_prime])
                | where(lambda _, s_prime: css_pcs[s] in css_pcs[s_prime]):
                    parents[s].append(s_prime)
                    children[s_prime].append(s)


def is_independent_in(pcs: tuple[pc], R_6):
    raise NotImplementedError()


def is_equal_facet(phi1: Facet, phi2: Facet) -> bool:
    raise NotImplementedError()


def v_space(supported_phi: tuple[pc]):
    raise NotImplementedError()


def support(phi: Facet) -> tuple[pc]:
    raise NotImplementedError()


def is_feasible1(phi: Facet, *pc_tuple: tuple[pc]) -> bool:
    raise NotImplementedError()


def is_feasible2(phi: Facet,
                 s1: ConfigurationState,
                 s2: ConfigurationState) -> bool:
    raise NotImplementedError()


def merge(pc1: pc, pc2: pc) -> pc:
    raise NotImplementedError()


def grouping_C_states(
    facets: "dict[int,Iterable[Facet]]",
    configuration_states: "dict[Facet,Iterable[ConfigurationState]]",
) -> "dict[ConfigurationState, Iterable[ConfigurationState]]":
    "Page 64, Figure 24"
    sibs: dict[ConfigurationState, list[ConfigurationState]] = dict()
    for i in range(4):
        for phi in facets[i]:
            for s in configuration_states[phi]:
                sibs[s] = []
        for s1, s2 in _pairs(configuration_states[phi]):
            if is_feasible2(phi, s1, s2):
                sibs[s1].append(s2)
                sibs[s2].append(s1)
    return sibs


T = TypeVar("T")


def _pairs(iter: Iterable[T]) -> Iterable[Tuple[T, T]]:
    "Return all pairs from a enumarable"
    return itertools.combinations(iter, 2)
