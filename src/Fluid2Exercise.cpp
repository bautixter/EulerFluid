#include "Scene.h"

#include "Numeric/PCGSolver.h"

namespace asa
{
namespace
{
////////////////////////////////////////////////
// Add any reusable classes or functions HERE //
////////////////////////////////////////////////
Vector2 clampToSafeDomain(Grid2 grid, Vector2 indexPair)
{
    Index2 limits = grid.getSize();

    Vector2 newIndexPair;
    
    // Upper bound
    newIndexPair.x = std::min(indexPair.x, float(limits.x - 1));
    newIndexPair.y = std::min(indexPair.y, float(limits.y - 1));

    // Lower bound
    newIndexPair.x = std::max(newIndexPair.x, 0.f);
    newIndexPair.y = std::max(newIndexPair.y, 0.f);

    return newIndexPair;
}

}  // namespace

// advection
void Fluid2::fluidAdvection(const float dt)
{
    // Copiar el estado original de inkRGB a inkRGB0
    Array2<Vector3> inkRGB0(inkRGB);

    // Iterar sobre cada celda en inkRGB
    for (int i = 0; i < inkRGB.getSize().x; ++i) {
        for (int j = 0; j < inkRGB.getSize().y; ++j) {
            // Obtener la posición central de la celda (i, j)
            Vector2 Xt = grid.getCellPos(Index2(i, j));

            // Obtener la velocidad en la celda (i, j)
            Vector2 Ut = Vector2(getVelocityX()[i, j], getVelocityY()[i, j]);

            // Calcular la posición anterior
            Vector2 Xt_prev = Xt - dt * Ut;

            // Obtener el índice de la celda correspondiente a la posición anterior
            Vector2 IJt_prev = grid.getCellIndex(Xt_prev);

            // Asegurarse de que el índice esté dentro del dominio seguro
            Vector2 IJt_safe = clampToSafeDomain(grid, IJt_prev);

            // Obtener las coordenadas fraccionales para la interpolación
            float tx = IJt_safe.x - floor(IJt_safe.x);
            float ty = IJt_safe.y - floor(IJt_safe.y);

            // Obtener los índices de los cuatro puntos más cercanos
            int i0 = static_cast<int>(floor(IJt_safe.x));
            int j0 = static_cast<int>(floor(IJt_safe.y));
            int i1 = i0 + 1;
            int j1 = j0 + 1;

            // Asegurarse de que los índices estén dentro de los límites
            i0 = std::max(0, std::min(i0, int(inkRGB0.getSize().x - 1)));
            j0 = std::max(0, std::min(j0, int(inkRGB0.getSize().y - 1)));
            i1 = std::max(0, std::min(i1, int(inkRGB0.getSize().x - 1)));
            j1 = std::max(0, std::min(j1, int(inkRGB0.getSize().y - 1)));

            // Obtener los valores de inkRGB0 en los cuatro puntos más cercanos
            Vector3 aa = inkRGB0[i0,j0];
            Vector3 ba = inkRGB0[i1,j0];
            Vector3 ab = inkRGB0[i0,j1];
            Vector3 bb = inkRGB0[i1,j1];

            // Realizar la interpolación bilineal
            Vector3 interpolatedValue = bilerp(aa, ba, ab, bb, tx, ty);

            // Asignar el valor interpolado a inkRGB[i][j]
            inkRGB[i, j] = interpolatedValue;
        }
    }

    {
        // Velocity acvection term HERE

        // Copy VelocityX to VelocityX0
        // Copy VelocityY to VelocityY1

        // Do SL for VelocityX
        // Do SL for VelocityY
    }
}

void Fluid2::fluidEmission()
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Emitters contribution HERE
        /*Emisor 1 Domain = (-0.1, -1.9), (0.1, -1.75);
        Ink = (1.0, 1.0, 0.0);
        Velocity = (0.0, 0.0);*/
    }
}

void Fluid2::fluidVolumeForces(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Gravity term HERE
    }
}

void Fluid2::fluidViscosity(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Viscosity term HERE
    }
}

void Fluid2::fluidPressureProjection(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Incompressibility / Pressure term HERE

        // foreach velocityX in border (L,R)
        //      velocityX[i,j] = 0
        // foreach velocityY in border (D,U)
        //      velocityY[i,j] = 0

        // Calculamos el vector derecho del sistema de ecuaciones
        // -------------------------------------------------------
        // DoF = Degrees of Freedom -> NCellsX * NCellsY
        // Vector<double> RHS (DoF)
        // foreach cell
        //   cellRHS = Compute RHS multiplied by constants <- divergencia * constante (diapo 44)
        //   RHS[getLinearIndex(i,j)] = cellRHS
        
        // Calculamos la matriz izquierda del sistema de ecuaciones
        // -------------------------------------------------------
        // SparseMatrix<double> LHS (DoF, 5)
        // foreach cell
        //   indexDiag = linearIndex(i,j)
        //   indexL = linearIndex(i-1, j)
        //   indexR = linearIndex(i+1, j)
        //   indexU = linearIndex(i, j+1)
        //   indexD = linearIndex(i, j-1)
        //   ...sin tener en cuenta condiciones de contorno
        //   A(indexDiag, indexDiag) = -1 * (2 / Dx2 - 2 / Dy2)
        //   Fill offdiag terms
        //   ...las condiciones de contorno afectan a la diagonal
        //   nos iguala aquello que cae fuera a lo que cae en el borde
        //   borde L ó R: -1/ Dx2
        //   borde U ó D: -1/ Dy2
        
        // Notas
        // - Cada fila de la matriz LHS representa la ecuación de una celda
        // - Tendremos que ciertas filas, representarán celdas en los bordes, para los cuales aplicamos condiciones de
        // contorno

        // Resolvemos el sistema de ecuaciones
        // -----------------------------------
        // Vector<double> ps(NDoF)
        // Init all Ps to 0
        // PCGSolver<double> PCG
        // PCG set_solver_parameters 1e-5, 200
        // PCG.solve(LHS, RHS, ps)
        // residual, iterations
        // PCG solve (A, RHS, Ps, residual, iterations)

        // Foreach NDoFs
        //   pressure[i,j] = static_cast<float>(Ps[Dof])

        // Actualizamos las velocidades
        // ----------------------------
        // Foreach velocityX (si la celdano es borde)
        //   if celda no borde:
        //      velocityX[i,j] -= grad P wrt x
        //      ...esto es, cojo la presión de la celda derecha y la de la izquierda y divido por dx
        // Foreach velocityY (que no es borde)
        //   if celda no borde:
        //      velocityY[i,j] -= grad P wrt y
        //      ...esto es, cojo la presión de la celda de arriba y la de abajo y divido por dy

    }
}
}  // namespace asa
