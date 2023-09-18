import meshCreation
import argparse


def main():

    parser = argparse.ArgumentParser(description="Transform the vegetation inside a classified point cloud into a set of meshes")

    parser.add_argument("-i", "--input", help="Input las file (default ='./SampleDatas/ExampleDataIsolatedTrees.las')", default="./SampleDatas/ExampleDataIsolatedTrees.las")
    parser.add_argument("-o", "--output", help="Output directory (default ='./output/')", default="./output/")
    parser.add_argument("-c", "--cellsize", help="Cell size (default = 2.0)", default=2.0, type=float)
    parser.add_argument("-v", "--verbose", help="Increase output verbosity", action="store_true")

    args = parser.parse_args()

    input_path = args.input
    output_path = args.output
    cellSize = args.cellsize
    verbose = args.verbose

    meshCreation.vegetationToMesh(input_path, output_path, cellSize, verbose)

if __name__ == "__main__":
    main()