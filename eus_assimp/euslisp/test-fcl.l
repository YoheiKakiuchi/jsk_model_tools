(load "package://eus_assimp/euslisp/eus-assimp.l")

(let ((foreign-lib
       (load-foreign "libtest_fcl.so")))
  (defforeign raw-col-check foreign-lib "eus_col_check" (:string :integer :string :integer
                                                         :string :integer :string :integer
                                                         :string :string :string :string
                                                         :integer :integer :integer :integer) :float)
  )

(defun check-collision (obj1 obj2 relative-coords
                             &key
                             (return-distance 0)
                             (split-method 0)
                             (object-type 0)
                             (verbose 1))
  (let ((ids1 (cadr (assoc :indices (car (obj1 . gl::mesh-list)))))
        (ver1 (array-entity (cadr (assoc :vertices (car (obj1 . gl::mesh-list))))))
        (ids2 (cadr (assoc :indices (car (obj2 . gl::mesh-list)))))
        (ver2 (array-entity (cadr (assoc :vertices (car (obj2 . gl::mesh-list))))))
        (n1 (instantiate float-vector 3))
        (n2 (instantiate float-vector 3)))
    (case return-distance
      (0
       (raw-col-check
        ver1 (/ (length ver1) 3)
        ids1 (/ (length ids1) 3)
        ver2 (/ (length ver2) 3)
        ids2 (/ (length ids2) 3)
        (send relative-coords :pos)
        (matrix2quaternion (send relative-coords :rot))
        n1 n2
        return-distance object-type split-method verbose
        ))
      (t
       (let ((dist
              (raw-col-check
               ver1 (/ (length ver1) 3)
               ids1 (/ (length ids1) 3)
               ver2 (/ (length ver2) 3)
               ids2 (/ (length ids2) 3)
               (send relative-coords :pos)
               (matrix2quaternion (send relative-coords :rot))
               n1 n2
               return-distance object-type split-method verbose
               )))
         (list dist n1 n2))
       ))
    ))

(defun collision-viewer (&key (tm 100) (object-type 4) (split-method 0) (verbose 1))
  (setq glv1 (load-mesh-file "table.dae" :scale 1000.0))
  (setq glv2 (load-mesh-file "table.dae" :scale 1000.0))
  (setq pc (instance pointcloud :init
                     :points (list #f(-1000 0 0) #f(1000 0 0) #f(0 -1000 0)
                                   #f(0 1000 0) #f(0 0 -1000) #f(0 0 1000))))
  (dotimes (i tm)
    (let ((cds (make-coords :pos (scale 1000 (random-vector))
                            :rpy (scale pi (random-vector))))
          (points) (ln) dist)
      (setq points
            (check-collision glv1 glv2 cds
                             :split-method split-method
                             :object-type object-type
                             :verbose verbose
                             :return-distance 1))
      (setq dist (car points))
      (setq points (cdr points))

      (setq ln (apply #'make-line points))
      (send glv1 :reset-coords)
      (send glv1 :transform cds)
      (pprint dist)
      (if (<= dist 0.0)
          (send glv1 :set-color #f(0.8 0.0 0.0))
        (send glv1 :set-color #f(0.7 0.7 0.7)))
      ;;(send glv1 :mvoe-to cds :world)
      (objects (list glv1 glv2 pc))
      (do-until-key
       (send ln :draw-on :width 3.0 :color #f(0 0.7 0) :flush t)
       (x::window-main-one))
      )))
