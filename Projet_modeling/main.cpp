
#include <QApplication>
#include <QGLViewer/simple_viewer.h>
#include <matrices.h>
#include <primitives.h>
#include <meshquad.h>

const Vec3 ROUGE   = {1,0,0};
const Vec3 VERT    = {0,1,0};
const Vec3 BLEU    = {0,0,1};
const Vec3 JAUNE   = {1,1,0};
const Vec3 CYAN    = {0,1,1};
const Vec3 MAGENTA = {1,0,1};
const Vec3 BLANC   = {1,1,1};
const Vec3 GRIS    = {0.5,0.5,0.5};
const Vec3 NOIR    = {0,0,0};


void draw_repere(const Primitives& prim, const Mat4& tr)
{
    const Mat4 tr1 = tr * translate(0,0,0.2);
    prim.draw_sphere(tr1*scale(0.5,0.5,0.5), BLANC);

    //BLEU
    prim.draw_cylinder(
                tr1*translate(0,0,0.5)*scale(0.3,0.3,0.7),
                BLEU
    );

    prim.draw_cone(
                tr1*translate(0, 0, 1.1)*scale(0.5,0.5,0.6),
                BLEU
    );

    //VERT
    prim.draw_cylinder(
                tr1*rotateX(90)*translate(0,0,0.5)*scale(0.3,0.3,0.7),
                VERT
    );

    prim.draw_cone(
                tr1*rotateX(90)*translate(0, 0, 1.1)*scale(0.5,0.5,0.6),
                VERT
    );

    //ROUGE
    prim.draw_cylinder(
                tr1*rotateY(90)*translate(0,0,0.5)*scale(0.3,0.3,0.7),
                ROUGE
    );

    prim.draw_cone(
                tr1*rotateY(90)*translate(0, 0, 1.1)*scale(0.5,0.5,0.6),
                ROUGE
    );
}


void star(MeshQuad& m)
{
	m.create_cube();

    m.decale_quad(0,1);
    m.decale_quad(4,1);
    m.decale_quad(8,1);
    m.decale_quad(12,1);
    m.decale_quad(16,1);
    m.decale_quad(20,1);

    for(unsigned int i = 0; i < 7; i++)
    {
        float f = 1 - 0.05*i;
        float d = -0.1*i;

        m.extrude_quad(0);
        m.extrude_quad(4);
        m.extrude_quad(8);
        m.extrude_quad(12);
        m.extrude_quad(16);
        m.extrude_quad(20);

        m.decale_quad(0,d);
        m.decale_quad(4,d);
        m.decale_quad(8,d);
        m.decale_quad(12,d);
        m.decale_quad(16,d);
        m.decale_quad(20,d);

        m.shrink_quad(0,f);
        m.shrink_quad(4,f);
        m.shrink_quad(8,f);
        m.shrink_quad(12,f);
        m.shrink_quad(16,f);
        m.shrink_quad(20,f);

        m.tourne_quad(0,12);
        m.tourne_quad(4,12);
        m.tourne_quad(8,12);
        m.tourne_quad(12,12);
        m.tourne_quad(16,12);
        m.tourne_quad(20,12);
    }
}

void spirale_mesh(MeshQuad& m)
{
    m.create_cube();

    float f = 360/30;
    float f1 = 20, f2 = 5;

    m.decale_quad(0,f2);
    for(unsigned int i = 0; i < 120; i++)
    {
        if(i%10 == 0)
        {
            f2 = 5 - (i/20.0f);
        }

        m.extrude_quad(0);
        m.tourne_quad(28+16*i,f);
        m.tourne_quad(24+16*i,-f);
        m.tourne_quad(32+16*i,f1);
        m.tourne_quad(36+16*i,-f1);
        m.decale_quad(0,f2);
    }
}

void rec(MeshQuad& m, int quad)
{

    int q0 = quad;
    int q4 = quad+4;
    int q8 = quad+8;
    int q12 = quad+12;
    int q16 = quad+16;
    int q20 = quad+20;

    m.extrude_quad(q0);
    m.extrude_quad(q4);
    m.extrude_quad(q8);
    m.extrude_quad(q12);
    m.extrude_quad(q16);
    m.extrude_quad(q20);

//    m.shrink_quad(q0,0.5f);
//    m.shrink_quad(q4,0.5f);
//    m.shrink_quad(q8,0.5f);
//    m.shrink_quad(q12,0.5f);
//    m.shrink_quad(q16,0.5f);
//    m.shrink_quad(q20,0.5f);

//    m.extrude_quad(q0);
//    m.extrude_quad(q4);
//    m.extrude_quad(q8);
//    m.extrude_quad(q12);
//    m.extrude_quad(q16);
//    m.extrude_quad(q20);

//    m.decale_quad(q0,1);
//    m.decale_quad(q4,1);
//    m.decale_quad(q8,1);
//    m.decale_quad(q12,1);
//    m.decale_quad(q16,1);
//    m.decale_quad(q20,1);
}


void recursif(MeshQuad& m)
{

    m.create_cube();

    for(int i = 0; i < 1; i++)
    {
        rec(m,0);
        rec(m,4);
//        rec(m,8);
//        rec(m,12);
//        rec(m,16);
//        rec(m,20);
    }

}


int main(int argc, char *argv[])
{
	Primitives prim;
	int selected_quad = -1;
	glm::mat4 selected_frame;
	MeshQuad mesh;

	// init du viewer
	QApplication a(argc, argv);
	SimpleViewer::init_gl();
	SimpleViewer viewer({0.1,0.1,0.1},5);

	// GL init
	viewer.f_init = [&] ()
	{
		prim.gl_init();
		mesh.gl_init();
	};

	// drawing
	viewer.f_draw = [&] ()
	{
		mesh.set_matrices(viewer.getCurrentModelViewMatrix(),viewer.getCurrentProjectionMatrix());
		prim.set_matrices(viewer.getCurrentModelViewMatrix(),viewer.getCurrentProjectionMatrix());

		mesh.draw(CYAN);

		if (selected_quad>=0)
			draw_repere(prim,selected_frame);
	};

	// to do when key pressed
	viewer.f_keyPress = [&] (int key, Qt::KeyboardModifiers mod)
	{
		switch(key)
		{
			case Qt::Key_C:
				if (!(mod & Qt::ControlModifier))
					mesh.create_cube();
				break;

            case Qt::Key_E:
                if(selected_quad != -1)
                    mesh.extrude_quad(selected_quad);
                break;

            case Qt::Key_Up:
                if(selected_quad != -1)
                    mesh.decale_quad(selected_quad,1.0f);
                break;

            case Qt::Key_Down:
                if(selected_quad != -1)
                    mesh.decale_quad(selected_quad,-1.0f);
                break;

            case Qt::Key_Z:
                if(selected_quad != -1)
                {
                    if(mod & Qt::ShiftModifier)
                        mesh.shrink_quad(selected_quad,2.0f);
                    else
                        mesh.shrink_quad(selected_quad,0.5f);
                }

                break;

            case Qt::Key_T:
                if(selected_quad != -1)
                {
                    if(mod & Qt::ShiftModifier)
                        mesh.tourne_quad(selected_quad,1.0f);
                    else
                        mesh.tourne_quad(selected_quad,-1.0f);
                }

                break;

			// e extrusion
			// +/- decale
			// z/Z shrink
			// t/T tourne

			// Attention au cas m_selected_quad == -1

			// generation d'objet
			case Qt::Key_S:
                if(mod & Qt::ShiftModifier)
                    star(mesh);
                else
                    spirale_mesh(mesh);
				break;
            case Qt::Key_R:
                recursif(mesh);
                break;

			default:
				break;
		}

		Vec3 sc;
		float r;
		mesh.bounding_sphere(sc,r);
		viewer.setSceneCenter(qglviewer::Vec(sc[0],sc[1],sc[2]));
		viewer.setSceneRadius(r);
		viewer.camera()->centerScene();
		viewer.update();
	};

	// to do when mouse clicked (P + Dir = demi-droite (en espace objet) orthogonale à l'écran passant par le point cliqué
	viewer.f_mousePress3D = [&] (Qt::MouseButton /*b*/, const glm::vec3& P, const glm::vec3& Dir)
	{
		selected_quad = mesh.intersected_closest(P,Dir);
		if (selected_quad>=0)
			selected_frame = mesh.local_frame(selected_quad);
		std::cout << selected_quad << std::endl;
	};

	viewer.clearShortcuts();
	viewer.setShortcut(QGLViewer::EXIT_VIEWER,Qt::Key_Escape);
	viewer.show();
	return a.exec();
}
