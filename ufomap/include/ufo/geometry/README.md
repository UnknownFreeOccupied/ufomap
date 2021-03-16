Supports intersection test between
|                  | AABB | Capsule | Cone | Cylinder | Ellipsoid | Frustum | Line Segment | OBB | Plane | Point | Ray | Sphere | Triangle |
| ---------------- |:----:|:-------:|:----:|:--------:|:---------:|:-------:|:------------:|:---:|:-----:|:-----:|:---:|:------:|:--------:|
| **AABB**         | ✔    | ?       | ?    | ?        | ?         | ✔       | ✔            | ✔   | ✔     | ✔     | ✔   | ✔      | ?        |
| **Capsule**      | ?    | ?       | ?    | ?        | ?         | ?       | ?            | ?   | ?     | ?     | ?   | ?      | ?        |
| **Cone**         | ?    | ?       | ?    | ?        | ?         | ?       | ?            | ?   | ?     | ?     | ?   | ?      | ?        |
| **Cylinder**     | ?    | ?       | ?    | ?        | ?         | ?       | ?            | ?   | ?     | ?     | ?   | ?      | ?        |
| **Ellipsoid**    | ?    | ?       | ?    | ?        | ?         | ?       | ?            | ?   | ?     | ?     | ?   | ?      | ?        |
| **Frustum**      | ✔    | ?       | ?    | ?        | ?         | ✖       | ✖            | ✔   | ✖     | ✔     | ✖   | ✔      | ?        |
| **Line Segment** | ✔    | ?       | ?    | ?        | ?         | ✖       | ✖            | ✔   | ✔     | ✔     | ✖   | ✔      | ?        |
| **OBB**          | ✔    | ?       | ?    | ?        | ?         | ✔       | ✔            | ✔   | ✔     | ✖*    | ✔   | ✔      | ?        |
| **Plane**        | ✔    | ?       | ?    | ?        | ?         | ✖       | ✔            | ✖   | ✔     | ✔     | ✔   | ✔      | ?        |
| **Point**        | ✔    | ?       | ?    | ?        | ?         | ✔       | ✔            | ✔   | ✔     | ✔     | ✔   | ✔      | ?        |
| **Ray**          | ✔    | ?       | ?    | ?        | ?         | ✖       | ✖            | ✔   | ✔     | ✔     | ✖   | ✔      | ?        |
| **Sphere**       | ✔    | ?       | ?    | ?        | ?         | ✔       | ✔            | ✔   | ✔     | ✔     | ✔   | ✔      | ?        |
| **Triangle**     | ?    | ?       | ?    | ?        | ?         | ?       | ?            | ?   | ?     | ?     | ?   | ?      | ?        |

✔: means implemented
✖: not implemented
✖*: implemented but wrong
?: type does not even exist
