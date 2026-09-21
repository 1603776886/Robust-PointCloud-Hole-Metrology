# Reviewer Guide 鈥?Research Demo / Development Preview

> Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
> The public repository contains only the demonstration-oriented software components and selected reproducible examples.

This public repository intentionally contains only the selected **single-cloud / no-stitch Research Demo GUI**. It is not the complete engineering system, complete project deliverable, complete dataset, or an official software release of any funding programme or organization.

## Fast source-review entry points

- `apps/no_stitch/main.cpp`
- `apps/no_stitch/ZhuChuangKou_Window.*`
- `apps/no_stitch/HoleShibie_Recognition.*`
- `apps/no_stitch/HoleFaXian_Normal.h`
- `apps/no_stitch/HoleJihe_Geometry.*`
- `apps/no_stitch/HoleFenxi_Analysis.*`
- `apps/no_stitch/HoleWeizi_Pose.h`

## Fast execution path

For execution without a development environment, use the standalone Setup.exe attached to the GitHub Release. If public example point clouds are present under `examples/`, they may be used to exercise the demonstration workflow.

## Public/non-public boundary

See [`RESEARCH_DEMO_SCOPE.md`](RESEARCH_DEMO_SCOPE.md).