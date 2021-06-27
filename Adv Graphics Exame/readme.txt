

Notes:

The classes Quaternion, AxisAngle, Matrix3, Euler
are alternative ways to represent a rotation.

They expose many of the same methods (apply, inverse, lerp...).

It must be possible to use them interchangably (search source for "the BIG CHOICE").

There are "from" methods that convert one type to the next.

Your task is to complete the "TODO" methods.

* Keep track of the one you did in a done.txt file! Each TODO has a label. 
  E.g. if you implement "TODO Q-invert"
  then write down: "Q-Invert: DONE"

* Many methods can be completed my implementing the "from" method.
  E.g., to invert Axis-angles, convert them to matrix3, invert matrix3, convert back...
  (This includes CONVERSION METHODS!)
  If you do this, keep track. E.g. "Q-invert (via Matrix3)"

* Classes that interpolate badly (Matrix, Euler) have two interpolation methods;
  a directLerp, which intepolates badly,
  a lerp, which inerpolates nicely (convert to another format, interpolate, convert back)

In addition, file "problems.h" has simple problems with their own UNDO.

Please write more unittest to check that your methods are correct!

Ignore "matrix4.h". It is a rendundant file (for now).

