# Human skin SSS demo assets

Infinite, 3D Head Scan by Lee Perry-Smith, based on work at www.triplegangers.com. Licensed under [Creative Commons Attribution 3.0 Unported](https://creativecommons.org/licenses/by/3.0/).

The unmodified head mesh and its original license were downloaded from [three.js](https://github.com/mrdoob/three.js/tree/c47b9048674a43ab83983a64437d0820e0f8e620/examples/models/gltf/LeePerrySmith), revision `c47b9048674a43ab83983a64437d0820e0f8e620`. See `Assets/Models/Head/License.txt`. Source geometry contains 17,684 triangles and is closed after welding coincident UV-seam vertices; every geometric edge has two incident triangles.

The demo assigns a uniform skin material rather than photographed albedo, so the sculpt and scattering can be inspected directly. `Skin.evesss` is an artistic homogeneous diffusion preset, not a calibrated biological skin model. RGB diffusion distances are 1.2/0.5/0.25 mm and surface IOR is 1.4.

The engine copies these assets into the ignored generated demo project. Edit and save the generated profile to experiment; restarting the demo preserves profile edits. Material changes can be saved with the scene. The source asset files remain unchanged.
