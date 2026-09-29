# r2s_pipeline

Object physical parameters for simulation, from a robot's own sensors:

    mass, CoM  <- wrist wrench or joint torque, any motion     (GT-validated: <1 % mass, ~1 cm CoM)
    inertia    <- reconstructed geometry                       (GT-validated: 4.4 % median)

Inertia is deliberately NOT taken from torque: on ground truth it is 300-800 % wrong under any
production-realistic motion, and physically impossible on real objects. See ../STATUS.md.

## Install (this machine)
    conda env: s2s  (Drake 1.51, numpy 2.2, scipy, trimesh, lxml)
    run with:  PYTHONNOUSERSITE=1  and PYTHONPATH unset      e.g.
    env -u PYTHONPATH PYTHONNOUSERSITE=1 /home/kneepolean/miniconda3/envs/s2s/bin/python -m r2s_pipeline selftest

## Use
    python -m r2s_pipeline selftest
    python -m r2s_pipeline identify rec.npz --mesh obj.obj --baseline empty.npz --out obj.json
    python -m r2s_pipeline identify rec.npz --mode joint_torque --robot arm.urdf --ee flange --baseline empty.npz --mesh obj.obj

Python:
    from r2s_pipeline import identify, load_recording, sdf_inertial_block
    res = identify(load_recording("rec.npz"), mesh_path="obj.obj", baseline=load_recording("empty.npz"))

What to record: ../RECORDING_SPEC.md

## Layout
    regressor.py   Newton-Euler regressor, parameter pack/unpack, LS and physically-consistent SDP fits
    kinematics.py  sensor-frame kinematics from poses; automatic wrench frame/sign detection
    geometry.py    voxel inertia from a mesh (open meshes OK), gyration ratio, symmetry flag
    identify.py    wrench and joint-torque modes, baseline subtraction, held-out diagnostics
    io.py          .npz loader/validator, JSON, SDF/URDF inertial blocks, SDF patching
    cli.py         command line + hardware-free self-test on ground-truth data

## Validation record
    self-test (utias, wrench):        mass 0.06-1.8 %, 5/5 objects
    SR2S spam (joint torque, nominal iiwa + baseline):  0.366 kg vs 0.378 kg reference (3.2 %)
