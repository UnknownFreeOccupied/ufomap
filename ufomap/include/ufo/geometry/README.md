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

✔: means implemented<br>
✖: not implemented<br>
✖*: implemented but wrong<br>
?: type does not even exist
