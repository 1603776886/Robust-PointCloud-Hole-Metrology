# Reviewer guide

> **Research Demo / Development Preview**  
> Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
> The public repository contains only the demonstration-oriented software components and selected reproducible examples.

This public tree intentionally publishes only the selected single-cloud/no-stitch GUI implementation. There is no stitching module and no CLI/batch harness in the public demo.

Fast source-review entry points:

- `apps/no_stitch/main.cpp`
- `apps/no_stitch/ZhuChuangKou_Window.*`
- `apps/no_stitch/HoleShibie_Recognition.*`
- `apps/no_stitch/HoleFaXian_Normal.h`
- `apps/no_stitch/HoleJihe_Geometry.*`
- `apps/no_stitch/HoleFenxi_Analysis.*`
- `apps/no_stitch/HoleWeizi_Pose.h`

For execution without a development environment, use the Setup.exe attached to the GitHub Release rather than rebuilding the source.

See `RESEARCH_DEMO_SCOPE.md` for the public/non-public boundary.
