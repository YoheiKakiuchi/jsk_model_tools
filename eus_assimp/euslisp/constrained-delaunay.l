(load "constrained_delaunay.so")

#|
(setq cb (make-cube 1000 1000 50))
(setq cyl (make-cylinder 100 100 :segments 5))
(send cyl :translate #f(0 0 -50))
(setq cb (body- cb cyl))
(setq fs (find-if #'(lambda (x) (eq (car (send x :id)) :top)) (send cb :faces)))

(setq cb (make-cube 200 200 50))
(setq cb2 (make-cube 190 210 50))
(send cb2 :translate #f(0 0 -10))
(setq cb (body- cb cb2))
(setq fs (elt (send cb :faces) 1))
|#
(defun constrained-delaunay (aface &optional (eps 1e-6))
  (let ((nm (send aface :normal))
        (pos (car (send aface :vertices)))
        cds (edges) (del-res) ret
        )
    (when
        (or (c-isnan (elt nm 0))
            (c-isnan (elt nm 1))
            (c-isnan (elt nm 2)))
      (return-from constrained-delaunay nil))
    (setq cds (orient-coords-to-axis (make-coords) nm :z eps))
    (send cds :locate pos :world)

    (dolist (e (send aface :edges))
      (push e edges))

    (dolist (h (send aface :holes))
      (dolist (e (send h :edges))
        (push e edges)))
    (setq edges (reverse edges))
    (let* ((len (length edges))
           (fv (instantiate float-vector (* 2 2 len)))
           (cntr 0)
           )
      (dolist (e edges)
        (let ((pv (send cds :inverse-transform-vector (e . pvert)))
              (nv (send cds :inverse-transform-vector (e . nvert))))
          ;;(pprint (list pv nv))
          (setf (elt fv cntr)       (elt pv 0))
          (setf (elt fv (+ cntr 1)) (elt pv 1))
          (setf (elt fv (+ cntr 2)) (elt nv 0))
          (setf (elt fv (+ cntr 3)) (elt nv 1))
          (incf cntr 4)
          ))
      ;;(pprint fv)
      (setq del-res (c-constrained-delaunay-triangulation fv))
      )
    (let* ((len (length del-res))
           (mat
            (make-matrix (/ len 3) 3)))
      (fvector-replace (array-entity mat) del-res)
      (c-coords-transform-vector (send cds :worldpos) (send cds :worldrot) mat mat)
      (dotimes (i (/ len 9))
        (let* ((v0 (c-matrix-row mat (+ (* i 3) 0)))
               (v1 (c-matrix-row mat (+ (* i 3) 1)))
               (v2 (c-matrix-row mat (+ (* i 3) 2)))
               (vlst (list v0 v1 v2))
               (holes (send aface :holes))
               )
          #|
          (let ((p (instance pointcloud :init :points (list v0 v1 v2))))
            (send p :point-size 3)
            (send *irtviewer* :objects (list p (instance faceset :init :faces (list aface))))
            (send *irtviewer* :draw-objects)
            )
          |#
          (unless
              (some #'(lambda (h)
                        (every #'(lambda (p) (eq :border (send h :insidep p eps))) (list v0 v1 v2)))
                    holes)
            ;;(pprint 'a)
            (let ((m0 (midpoint 0.5 v0 v1))
                  (m1 (midpoint 0.5 v1 v2))
                  (m2 (midpoint 0.5 v2 v0)))
              ;;(pprint (mapcar #'(lambda (p) (send aface :insidep p eps)) (list m0 m1 m2)))
              (unless
                  (some #'(lambda (p) (eq :outside (send aface :insidep p eps))) (list m0 m1 m2))
                ;;(pprint 'b)
                (push (make-face-from-vertices (list (list v0) (list v1) (list v2))) ret))
              ))
          ;;(do-until-key (x::window-main-one))
          ))
      )
    ret))
;;
;; replace triangulation
;;
(in-package "GEOMETRY")
(defun face-to-triangle-aux (aface)
  (if (null aface) nil
    (user::constrained-delaunay aface)))
(defun face-to-triangle (aface)
  (if (null aface) nil
    (user::constrained-delaunay aface)))
(defun body-to-faces (abody)
  "return triangled faces of given body"
  (body-to-triangles abody nil))
(defun body-to-triangles (abody &optional (limit 50))
  (let (fs)
    (setq *triangle-points* nil)
    (setq *triangle-edges* nil)
    (dolist (f (send abody :faces))
      (setq fs
            (nconc
             fs
             (mapcan #'(lambda (f)
                         (if limit
                             (geo::triangle-to-triangle f limit)
                           (list f)))
                     (geo::face-to-triangle f)))))
    (instance faceset :init :faces fs)))
(in-package "USER")
