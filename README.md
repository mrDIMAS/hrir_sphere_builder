# HRTF Builder

Special tool to make HRIR sphere mesh from IRCAM http://recherche.ircam.fr/equipes/salles/listen/download.html base.

## Usage

`hrtf_sphere_builder <directory_path>`

Output file will have this name: `directory_name.bin` and will be placed in parent directory of `directory_path`.

## Why it is needed?

HRIR database sampled at fixed angles - in case of IRCAM it has 15 degress azimuthal resolution and variable elavation resolution. Such fact gives annoying "jumping" effect of sound source spatial positioning. Resolution can be effectively increased by interpolation of HRIR's from three closest points for given source's azimuth and elevation. Working in spherical coordinates is error prone and may lead to various bugs, to bypass this we can use cartesian coordinate system. To be able to work in cartesian coordinates we need to know triangulated surface of a HRIR sphere so we can find a triangle which is intersected by ray from listener to sound source position. Once we have a triangle, we can use barycentric coordinates of a ray's intersection point to get weights for interpolation.

## How it works?

The tool reads file-by-file from database and extracts azimuth and elevation from file name, then tool translates spherical coordinates to cartesion and adds hrir point to point cloud. Each file contains head-related impulse response for left and right ears. Each channel gets converted into array of floats in [0; 1] range (normalized range). After all points were created, tool creates convex hull using these points. Finally, triangulated hrir sphere saved to disk.

## File format

The tool produces binary file, first goes header:

| Field        | Size | Type     | Value |
|--------------|------|----------|-------|
| magic        | 4    | uint32_t | HRIR  |
| sample_rate  | 4    | uint32_t |       |
| length       | 4    | uint32_t |       |
| vertex_count | 4    | uint32_t |       |
| index_count  | 4    | uint32_t |       |

Then goes indices:

| Field   | Size            | Type     |
|---------|-----------------|----------|
| Indices | 4 * index_count | uint32_t |

Finally goes `vertex_count` vertices, each vertex has this format:

| Field      | Size       | Type  |
|------------|------------|-------|
| X          | 4          | float |
| Y          | 4          | float |
| Z          | 4          | float |
| Left HRIR  | 4 * length | float |
| Right HRIR | 4 * length | float |

## How to use produced sphere?

- Translate vector from listener position to sound source position into listener coordinate system (by multiplying vector with listener view matrix)
- Use this vector to find an intersection point and intersected triangle with sphere mesh
- Calculate barycentric coordinates of intersection point 
- Use barycentric coordinates as weights for HRTF samples (as reference you can use this paper - http://www02.smt.ufrj.br/~diniz/conf/confi117.pdf)

## How to build the tool?

`g++ -o hrir_sphere_builder hrir_sphere_builder.cpp`