# Maize Tassel L-System

Monopodial tassel with lateral non-branching branches terminated by spikelets.

## Structure

- **Main rachis**: Central axis produced by `TasselApex` consuming vigor
- **Laterals**: Non-branching side branches at each internode
- **Spikelets**: Terminal organs on lateral tips and at rachis terminus

## Usage

1. Load `MaizeTasselDescriptor` asset (`.mtassel`) in EvoEngine editor
2. Adjust parameters (vigor, angles, lengths)
3. Derive to generate the graph
4. Geometry is produced via `GeometryPass` + cylinder mesh generation
