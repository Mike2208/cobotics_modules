import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy
import matplotlib.pyplot as plt


def get_program_parameters():
    import argparse
    description = 'Read a polydata file.'
    epilogue = ''''''
    parser = argparse.ArgumentParser(description=description, epilog=epilogue,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('filename', help='Torso.vtp')
    args = parser.parse_args()
    return args.filename


if __name__ == '__main__':
    colors = vtk.vtkNamedColors()

    filename = "./Geometry/arm_r_humerus.vtp"  # get_program_parameters()

    # Read all the data from the file
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(filename)
    reader.Update()

    # Visualize
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(colors.GetColor3d('NavajoWhite'))

    nodes_vtk_array = reader.GetOutput().GetPoints().GetData()
    polys = reader.GetOutput().GetPolys().GetData()


    nodes_nummpy_array = vtk_to_numpy(nodes_vtk_array)
    x, y, z = nodes_nummpy_array[:,0] , nodes_nummpy_array[:,1] , nodes_nummpy_array[:,2]

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(x, y, z)
    plt.show()
    # renderer = vtk.vtkRenderer()
    # renderWindow = vtk.vtkRenderWindow()
    # renderWindow.AddRenderer(renderer)
    # renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    # renderWindowInteractor.SetRenderWindow(renderWindow)
    #
    # renderer.AddActor(actor)
    # renderer.SetBackground(colors.GetColor3d('DarkOliveGreen'))
    # renderer.GetActiveCamera().Pitch(90)
    # renderer.GetActiveCamera().SetViewUp(0, 0, 1)
    # renderer.ResetCamera()
    #
    # renderWindow.SetSize(600, 600)
    # renderWindow.Render()
    # renderWindow.SetWindowName('ReadPolyData')
    # renderWindowInteractor.Start()
    # time.sleep(5)
    # renderWindowInteractor.Stop()