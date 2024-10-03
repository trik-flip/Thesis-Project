# Fine-Motion_Planning_For_Robotic_Assembly_In_Local
[page 59]
an adjacency graph of contact states is constructed by enumerating sub-tuples of contact space vertices' 6-tuples of PCs, giving rase to the multidimensional facets and contact states of contact space.
Arcs are placed in the graph to represent the spatial adjacency of contact state points sets in R^6.
As shown in figure 21, the contact state adjacency graph provides a search structure for exploring the multidimensional contact states that constutute the local contact space.
the following sections describe procedures for constructing contact state adjacency graphs.
## adjacency graph: one C-state per facet
Once the vertices of a local contact space have been identified and located, the higher-dimentsional contact states can be enumerated and placed in an adjacency graph. 
For some sasemblies, such as the rectangular peg-in-hole assembly, the imposition of artificial C-space limits leads to a relatively simple adjacency graph construction. 
Through the use of appropriate rotational constraints, such as delta x < 0, delta y < 0 for the square peg-in-hole assembly, the line and surface contacts of contact space may be eliminated. 
The remaining contact states are comprosed of "pure" point contact combinations. 
In such contact space, the formation or separation of each PC is generally associated with a change in contact space facets dimensionality, so there is normally a one-to-one correspondence between contact space facets and contact states. 
This leads to a relatively simple procedure for constructing a contact state adjacency graph
# Algo 1 - Figure 23
```python
for i in range(4):
    facets = []
    for facet in facets:
        children = []
        for pc_tuple in facet_pcs(facet):
            if is_independent_in(c_surface_of(pc_tuple),R_6):
                facet_prime = None
                facet_pcs(facet_prime) = pc_tuple
                support(facet_prime)
                if is_equal_facet(facet_prime, facet_double_prime) for 'some' facet_double_prime in facets:
                    facet_pc_prime = merge(facet_pcs(facet_prime), facet_pcs(facet_double_prime))
                    parents.add(facet_prime)
                else:
                    facets.add(facet_prime)
    for facet in facets:
        cstates = [] # cstates(facets)
        for pc_tuple in facet_pcs(facet):
            if is_feasible(facet, pc_tuple) and v_space(pc_tuple) == v_space(support(facet)):
                S:C_State = None
                cstate_pcs(S) = pc_tuple
                cstates(facet).add(S)
                for facet_prime in parents(facet):
                    for S_prime:ContactState in cstates(facet_prime):
                        if cstates_pcs(S) in cstates_pcs(S_prime)
                            parents(S).add(S_prime)
                            children(S_prime).add(S)
```
```julia
for i = 0:4
    facets[i+1] = []
    for ϕ::Facet in facets[i]:
        children[ϕ] = []
        for pc_tuple in facet_pcs(facet):
            if is_independent_in(c_surface_of(pc_tuple),R_6):
                facet_prime = None
                facet_pcs(facet_prime) = pc_tuple
                support(facet_prime)
                if is_equal_facet(facet_prime, facet_double_prime) for 'some' facet_double_prime in facets:
                    facet_pc_prime = merge(facet_pcs(facet_prime), facet_pcs(facet_double_prime))
                    parents.add(facet_prime)
                else:
                    facets.add(facet_prime)
    for facet in facets:
        cstates = [] # cstates(facets)
        for pc_tuple in facet_pcs(facet):
            if is_feasible(facet, pc_tuple) and v_space(pc_tuple) == v_space(support(facet)):
                S:C_State = None
                cstate_pcs(S) = pc_tuple
                cstates(facet).add(S)
                for facet_prime in parents(facet):
                    for S_prime:ContactState in cstates(facet_prime):
                        if cstates_pcs(S) in cstates_pcs(S_prime)
                            parents(S).add(S_prime)
                            children(S_prime).add(S)
end
```