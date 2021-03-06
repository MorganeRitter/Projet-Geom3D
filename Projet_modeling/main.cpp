
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
        for(unsigned int j = 0; j<21; j+=4)
        {
            m.extrude_quad(j);

            m.decale_quad(j,d);

            m.shrink_quad(j,f);

            m.tourne_quad(j,12);
        }
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


void recursif(MeshQuad& m,int q, int h_c, int h_max)
{
    int qface = q;
    int size = 0;
    if(h_c < h_max)
    {
        m.extrude_quad(qface);
        m.shrink_quad(qface,0.5f);
        m.extrude_quad(qface);
        m.decale_quad(qface,1.2f);
        m.extrude_quad(qface);
        size = m.nb_quads()*4;
        recursif(m,qface,h_c+1,h_max);
        recursif(m,size-4,h_c+1,h_max);
        recursif(m,size-8,h_c+1,h_max);
        recursif(m,size-12,h_c+1,h_max);
        recursif(m,size-16,h_c+1,h_max);
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
                {
                    mesh.extrude_quad(selected_quad);
                    mesh.gl_update();
                }
                break;

            case Qt::Key_Up:
                if(selected_quad != -1)
                {
                    mesh.decale_quad(selected_quad,1.0f);
                    mesh.gl_update();
                }
                break;

            case Qt::Key_Down:
                if(selected_quad != -1)
                {
                    mesh.decale_quad(selected_quad,-1.0f);
                    mesh.gl_update();
                }
                break;

            case Qt::Key_Z:
                if(selected_quad != -1)
                {
                    if(mod & Qt::ShiftModifier)
                    {
                        mesh.shrink_quad(selected_quad,2.0f);
                        mesh.gl_update();
                    }
                    else
                    {
                        mesh.shrink_quad(selected_quad,0.5f);
                        mesh.gl_update();
                    }
                }

                break;

            case Qt::Key_T:
                if(selected_quad != -1)
                {
                    if(mod & Qt::ShiftModifier)
                    {
                        mesh.tourne_quad(selected_quad,1.0f);
                        mesh.gl_update();
                    }
                    else
                    {
                        mesh.tourne_quad(selected_quad,-1.0f);
                        mesh.gl_update();
                    }
                }

                break;

			// generation d'objet
			case Qt::Key_S:
                if(mod & Qt::ShiftModifier)
                {
                    star(mesh);
                    mesh.gl_update();
                }
                else
                {
                    spirale_mesh(mesh);
                    mesh.gl_update();
                }
				break;
            case Qt::Key_R:
                mesh.create_cube();
                recursif(mesh,0,0,8);
                mesh.gl_update();
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
