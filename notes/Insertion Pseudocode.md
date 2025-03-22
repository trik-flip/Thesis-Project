# Insertion Pseudocode
## 1) Reaching the hole plane
Current constraints: None
Target constraint: (+fz , 1.5N)
Implicit CF targets: (CF1) point or edge on face
Additional motions:
- Lateral exploration within the workspace area, ran-
domly selecting x/y direction within workspace limits.
- In-hand manipulation to tilt the object toward the
direction of the lateral motion, to ensure an edge/face
contact, and avoid a face/face contact.
## 2) Searching for the hole
Current constraints: (+fz , 1.5N)
Target constraint: (+fx, 0.7N)
Implicit CF target: (CF2) 3-point contact with hole
Additional motions: Lateral exploration as previous step.
## 3) Wedging
Current constraints: (+fz , 1.5N), (+fx, 0.7N)
Target constraint: (+fy , 0.7N)
Implicit CF target: (CF3) 4-point contact with hole
## 4) Rotational alignment of peg and hole
Current constraints: (+fz , 1.5N), (+fx, 0.7N), (+fy ,
0.7N)
Target constraint: (+τz , 0.1N/m)
Implicit CF target: (CF4) hinge-type contact
Note: This step applies only to non-cylindrical objects.
## 5) Correcting upward tilt
Current constraints: (+fz , 1.5N)
Target constraint: (+fx, 0N), (+fy , 0N)
Implicit CF target: (CF5) Prismatic joint-type contact
Additional motion: Rotation around x-and y-axes
to minimize the accumulated angle between the object
and the hand due to fingertip slip during the previous
steps. This rotation is performed both with in-hand
manipulation and arm motions.
Note: The lateral forces are now minimized to avoid
jamming the object. This also helps centering the object
in the hole if the peg rotation is not perfectly centered
on the object's center of mass. The angle information is
provided by extracting the 3D pose of a marker placed on
the surface of the object as seen from the palm camera.
## 6) Inserting peg
Current constraints: (+fz , 1.5N), (+fx, 0N), (+fy , 0N)
Exit condition: When the fingertips start touching the
hole surface or the object hits the hole bottom, we
switch to disengagement. This can be detected as a
sharp increase in fz .
Note: We have already reached the CF state that enables
peg insertion, thus the system maintains it while the final
free DOF, i.e., the z-axis translation, is controlled to
perform the final insertion motion.
## 7) Detaching hand grip and retracting arm
Action: The hand opens and the arm returns to its origin
position in an open loop motion
