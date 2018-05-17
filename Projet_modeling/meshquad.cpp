#include "meshquad.h"
#include "matrices.h"


void MeshQuad::clear()
{
    //à vérifier
    m_points.clear();
    m_quad_indices.clear();
    //edges.clear(); ?
}

int MeshQuad::add_vertex(const Vec3& P)
{
    m_points.push_back(P);
    return (m_points.size() - 1);
}


void MeshQuad::add_quad(int i1, int i2, int i3, int i4)
{
    m_quad_indices.push_back(i1);
    m_quad_indices.push_back(i2);
    m_quad_indices.push_back(i3);
    m_quad_indices.push_back(i4);
}

void MeshQuad::convert_quads_to_tris(const std::vector<int>& quads, std::vector<int>& tris)
{
    tris.clear();
	tris.reserve(3*quads.size()/2);

	// Pour chaque quad on genere 2 triangles
	// Attention a repecter l'orientation des triangles
    //ne marche pas si pas un multiple de 4
    for( unsigned int i=0; i<quads.size(); i+=4) {
        tris.push_back(quads[i]);
        tris.push_back(quads[i+1]);
        tris.push_back(quads[i+2]);

        tris.push_back(quads[i]);
        tris.push_back(quads[i+2]);
        tris.push_back(quads[i+3]);
    }
}

void MeshQuad::convert_quads_to_edges(const std::vector<int>& quads, std::vector<int>& edges)
{
	edges.clear();
    edges.reserve(quads.size()*2); // ( *2 /2 !)
	// Pour chaque quad on genere 4 aretes, 1 arete = 2 indices.
	// Mais chaque arete est commune a 2 quads voisins !
	// Comment n'avoir qu'une seule fois chaque arete ?

    //ne marche pas si pas un multiple de 4
    for(unsigned int i=0; i<quads.size(); i+=4) {
        edges.push_back(quads[i]);
        edges.push_back(quads[i+1]);

        edges.push_back(quads[i+2]);
        edges.push_back(quads[i+3]);

        edges.push_back(quads[i+3]);
        edges.push_back(quads[i]);

        edges.push_back(quads[i+1]);
        edges.push_back(quads[i+2]);
    }
}


void MeshQuad::bounding_sphere(Vec3& C, float& R)
{
    // C = Moyenne de tous les vertex
    // R = Maximum du parcours de tout les vertex -> calcule de la distance entre centre et chaque vertex

    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    for(unsigned int i = 0; i <m_points.size(); i++)
    {
        x += m_points[i].x;
        y += m_points[i].y;
        z += m_points[i].z;
    }
    //std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;

    x = x/m_points.size();
    y = y/m_points.size();
    z = z/m_points.size();

    std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;

    C.x = x;
    C.y = y;
    C.z = z;

    float max = 0.0;

    for(unsigned int i = 0; i <m_points.size(); i++)
    {
        float comparaison = abs((C.x*C.x - m_points[i].x*m_points[i].x) + (C.y*C.y - m_points[i].y*m_points[i].y) + (C.z*C.z - m_points[i].z*m_points[i].z));
        if( comparaison > max)
        {
            max = comparaison;
        }
    }

    std::cout << max << " " <<sqrt(max) << std::endl;

    R = sqrt(max);
}


void MeshQuad::create_cube()
{
	clear();
	// ajouter 8 sommets (-1 +1)

    int pt0 = add_vertex(Vec3(0,1,0));
    int pt1 = add_vertex(Vec3(0,0,0));
    int pt2 = add_vertex(Vec3(1,0,0));
    int pt3 = add_vertex(Vec3(1,1,0));
    int pt4 = add_vertex(Vec3(1,1,-1));
    int pt5 = add_vertex(Vec3(0,1,-1));
    int pt6 = add_vertex(Vec3(0,0,-1));
    int pt7 = add_vertex(Vec3(1,0,-1));

	// ajouter 6 faces (sens trigo)

    add_quad(pt0,pt1,pt2,pt3); //devant -> 0
    add_quad(pt5,pt6,pt1,pt0); //gauche -> 4
    add_quad(pt5,pt0,pt3,pt4); //dessus -> 8
    add_quad(pt3,pt2,pt7,pt4); //droite -> 12
    add_quad(pt4,pt7,pt6,pt5); //derrière -> 16
    add_quad(pt1,pt6,pt7,pt2); //dessous -> 20

	gl_update();
}

Vec3 MeshQuad::normal_of(const Vec3& A, const Vec3& B, const Vec3& C)
{
	// Attention a l'ordre des points !
	// le produit vectoriel n'est pas commutatif U ^ V = - V ^ U
	// ne pas oublier de normaliser le resultat.
    Vec3 AB = Vec3(B.x-A.x,B.y-A.y,B.z-A.z);
    Vec3 AC = Vec3(C.x-A.x,C.y-A.y,C.z-A.z);

    Vec3 AB_AC = vec_cross(AB,AC);
    Vec3 Res = vec_normalize(AB_AC);
    return Res;
}

float MeshQuad::area_of_quad(int q)
{
    // recuperation des indices de points
    int quad1 = m_quad_indices[q];
    int quad2 = m_quad_indices[q+1];
    int quad3 = m_quad_indices[q+2];
    int quad4 = m_quad_indices[q+3];

    // recuperation des points
    Vec3 pt1 = m_points[quad1];
    Vec3 pt2 = m_points[quad2];
    Vec3 pt3 = m_points[quad3];
    Vec3 pt4 = m_points[quad4];

    //aire du premier triangle pt1-pt2-pt3

    float aire1 = vec_length(vec_cross(pt2-pt1,pt3-pt1))/2.0f;

    //aire du deuxième triangle pt1-pt3-pt4

    float aire2 = vec_length(vec_cross(pt3-pt1,pt4-pt1))/2.0f;

    return aire1+aire2;
}


bool MeshQuad::is_points_in_quad(const Vec3& P, const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
	// On sait que P est dans le plan du quad.

	// P est-il au dessus des 4 plans contenant chacun la normale au quad et une arete AB/BC/CD/DA ?
	// si oui il est dans le quad
    float res = -1;
    float d = 0;

    Vec3 normal = normal_of(A,B,C);
    Vec3 AB = B-A;
    Vec3 BC = C-B;
    Vec3 CD = D-C;
    Vec3 DA = A-D;

    //Plan n°1 AB

    Vec3 normal1 = vec_cross(normal,AB);
    d = vec_dot(normal1,A);
    res = vec_dot(normal1,P) - d;
    //std::cout << "plan 1 AB " << AB << " " << normal1 << " " << res << " " << d << std::endl;

    if(res < 0) return false;

    //Plan n°2 BC

    Vec3 normal2 = vec_cross(normal,BC);
    d = vec_dot(normal2,B);
    res = vec_dot(normal2,P) - d;

    //std::cout << "plan 2 Bc " << BC << " " << normal2 << " " << res <<" " << d<< std::endl;

    if(res < 0) return false;

    //Plan n°3 CD

    Vec3 normal3 = vec_cross(normal,CD);
    d = vec_dot(normal3,C);
    res = vec_dot(normal3,P) - d;

    //std::cout << "plan 3 CD " << CD << " " << normal3 << " " << res <<" " << d << std::endl;

    if(res < 0) return false;

    //Plan n°4 DA

    Vec3 normal4 = vec_cross(normal,DA);
    d = vec_dot(normal4,D);
    res = vec_dot(normal4,P) - d;

    //std::cout << "plan 4 DA " << DA << " " << normal4 << " " << res <<" " << d << std::endl;

    if(res < 0) return false;

    return true;
}

bool MeshQuad::intersect_ray_quad(const Vec3& P, const Vec3& Dir, int q, Vec3& inter)
{
    // recuperation des indices de points
    int quad1 = m_quad_indices[q];
    int quad2 = m_quad_indices[q+1];
    int quad3 = m_quad_indices[q+2];
    int quad4 = m_quad_indices[q+3];

    // recuperation des points
    Vec3 pt1 = m_points[quad1];
    Vec3 pt2 = m_points[quad2];
    Vec3 pt3 = m_points[quad3];
    Vec3 pt4 = m_points[quad4];

	// calcul de l'equation du plan (N+d)

    Vec3 normal = normal_of(pt1,pt2,pt3);

    float d = vec_dot(normal,pt1);


    // calcul de l'intersection rayon plan
	// I = P + alpha*Dir est dans le plan => calcul de alpha

    float t = (d-vec_dot(P,normal))/(vec_dot(Dir,normal));

    if( t == INFINITY)
        return false;
	// alpha => calcul de I

    Vec3 I = P+t*Dir;

    // I dans le quad ?

    if(is_points_in_quad(I,pt1,pt2,pt3,pt4))
    {
        inter = I;
        return true;
    }

    return false;
}


int MeshQuad::intersected_closest(const Vec3& P, const Vec3& Dir)
{
	// on parcours tous les quads

    int inter = -1;
    Vec3 vec_inter;
    Vec3 vec;
    float comparaison = 0.0;
    float min = 0;

    for(unsigned int i = 0; i<m_quad_indices.size(); i+=4)
    {
        // on teste si il y a intersection avec le rayon
        if(intersect_ray_quad(P,Dir,i,vec))
        {
//            std::cout << i << std::endl;
//            std::cout << min << std::endl;
//            std::cout << vec << std::endl;
            if(inter == -1)
            {
                inter = i;
                min = (vec.x-P.x)*(vec.x-P.x)+(vec.y-P.y)*(vec.y-P.y)+(vec.z-P.z)*(vec.z-P.z);
                vec_inter = vec;
            }
            else
            {
                comparaison = (vec.x-P.x)*(vec.x-P.x)+(vec.y-P.y)*(vec.y-P.y)+(vec.z-P.z)*(vec.z-P.z);
//                std::cout << comparaison << std::endl;
//                std::cout << min << std::endl;
                //on garde le plus proche (de P)
                if(abs(comparaison) < abs(min))
                {
                    inter = i;
                    min = comparaison;
                    vec_inter = vec;
                }
            }
        }
    }

	return inter;
}


Mat4 MeshQuad::local_frame(int q)
{
    /*
    Repere locale = Matrice de transfo avec
    les trois premieres colones: X,Y,Z locaux
    la derniere colonne l'origine du repere
    -> Mat4 = Vec4 X + Vec4 Y + Vec4 Z + Vec4 O
    ici Z = N et X = AB
    Origine le centre de la face
    longueur des axes : [AB]/2
    */

	// recuperation des indices de points
    int quad1 = m_quad_indices[q];
    int quad2 = m_quad_indices[q+1];
    int quad3 = m_quad_indices[q+2];
    int quad4 = m_quad_indices[q+3];

	// recuperation des points
    Vec3 pt1 = m_points[quad1];
    Vec3 pt2 = m_points[quad2];
    Vec3 pt3 = m_points[quad3];
    Vec3 pt4 = m_points[quad4];

    // calcul de Z: moyenne des normal des triangles /
    // X: normal du plan AB (A+Z) -> Y

    Vec3 Z = vec_normalize((normal_of(pt1,pt2,pt3)+normal_of(pt1,pt3,pt4))/2.0f);
    //std::cout << Z << std::endl;
    Vec3 X = vec_normalize(vec_cross(pt1-pt2,Z));
    //std::cout << X << std::endl;
    Vec3 Y = vec_normalize(vec_cross(X,Z));
    //std::cout << Y << std::endl;

	// calcul du centre

    Vec3 O = Vec3((pt1.x+pt2.x+pt3.x+pt4.x)/4,
                  (pt1.y+pt2.y+pt3.y+pt4.y)/4,
                  (pt1.z+pt2.z+pt3.z+pt4.z)/4);


    // calcul de la taille
    float taille = glm::length(pt2-pt1)/2;

    // calcul de la matrice

    Vec4 X4 = Vec4(X.x,X.y,X.z,0);
    Vec4 Y4 = Vec4(Y.x,Y.y,Y.z,0);
    Vec4 Z4 = Vec4(Z.x,Z.y,Z.z,0);
    Vec4 O4 = Vec4(O.x,O.y,O.z,1);

    //std::cout << X << Y << Z << O <<std::endl;

    Mat4 res = Mat4(X4,Y4,Z4,O4);
    res = res*scale(taille);

    //std::cout << res << std::endl;

    return res;
}

void MeshQuad::extrude_quad(int q)
{
	// recuperation des indices de points
    int quad1 = m_quad_indices[q];
    int quad2 = m_quad_indices[q+1];
    int quad3 = m_quad_indices[q+2];
    int quad4 = m_quad_indices[q+3];

    // recuperation des points
    Vec3 pt1 = m_points[quad1];
    Vec3 pt2 = m_points[quad2];
    Vec3 pt3 = m_points[quad3];
    Vec3 pt4 = m_points[quad4];

	// calcul de la normale

    Vec3 normal = normal_of(pt1,pt2,pt3);

	// calcul de la hauteur

    float hauteur = sqrt(area_of_quad(q));

	// calcul et ajout des 4 nouveaux points

    Vec3 A = pt1 + normal*hauteur;
    Vec3 B = pt2 + normal*hauteur;
    Vec3 C = pt3 + normal*hauteur;
    Vec3 D = pt4 + normal*hauteur;

    int n1 = add_vertex(A);
    int n2 = add_vertex(B);
    int n3 = add_vertex(C);
    int n4 = add_vertex(D);

    // on remplace les vertex quad initial par le quad du dessus -> nouveau vertex

    m_quad_indices[q] = n1;
    m_quad_indices[q+1] = n2;
    m_quad_indices[q+2] = n3;
    m_quad_indices[q+3] = n4;

	// on ajoute les 4 quads des cotes

    add_quad(n2,quad2,quad3,n3); //+4
    add_quad(n4,quad4,quad1,n1); //+8
    add_quad(n3,quad3,quad4,n4); //+12
    add_quad(n1,quad1,quad2,n2); //+16

   //gl_update();
}

void MeshQuad::transfo_quad(int q, const glm::mat4& tr)
{
	// recuperation des indices de points
    int quad1 = m_quad_indices[q];
    int quad2 = m_quad_indices[q+1];
    int quad3 = m_quad_indices[q+2];
    int quad4 = m_quad_indices[q+3];

    // recuperation des (references de) points

    Vec3& pt1 = m_points[quad1];
    Vec3& pt2 = m_points[quad2];
    Vec3& pt3 = m_points[quad3];
    Vec3& pt4 = m_points[quad4];

	// generation de la matrice de transfo globale:
	// indice utilisation de glm::inverse() et de local_frame
    Mat4 local = local_frame(q);

    float det = glm::determinant(local);
    if(det == 0)
    {
        std::cerr << "Erreur déterminant égale à 0 (quad n°" <<q<< ")"<<std::endl;
        return;
    }


    Mat4 transfo =local*tr*glm::inverse(local);

	// Application au 4 points du quad

    pt1 = Vec3(transfo * Vec4(pt1.x,pt1.y,pt1.z,1));
    pt2 = Vec3(transfo * Vec4(pt2.x,pt2.y,pt2.z,1));
    pt3 = Vec3(transfo * Vec4(pt3.x,pt3.y,pt3.z,1));
    pt4 = Vec3(transfo * Vec4(pt4.x,pt4.y,pt4.z,1));
}

void MeshQuad::decale_quad(int q, float d)
{

    Mat4 transfo = translate(0,0,d);
    transfo_quad(q,transfo);
    //gl_update();
}

void MeshQuad::shrink_quad(int q, float s)
{
    Mat4 transfo = scale(s);
    transfo_quad(q,transfo);
    //gl_update();
}

void MeshQuad::tourne_quad(int q, float a)
{
    Mat4 transfo = rotateZ(a);
    transfo_quad(q,transfo);
    //gl_update();
}





MeshQuad::MeshQuad():
	m_nb_ind_edges(0)
{}


void MeshQuad::gl_init()
{
	m_shader_flat = new ShaderProgramFlat();
	m_shader_color = new ShaderProgramColor();

	//VBO
	glGenBuffers(1, &m_vbo);

	//VAO
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_flat->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_flat->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);

	glGenVertexArrays(1, &m_vao2);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_color->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_color->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);

	//EBO indices
	glGenBuffers(1, &m_ebo);
	glGenBuffers(1, &m_ebo2);
}

void MeshQuad::gl_update()
{
	//VBO
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * m_points.size() * sizeof(GLfloat), &(m_points[0][0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	std::vector<int> tri_indices;
	convert_quads_to_tris(m_quad_indices,tri_indices);

	//EBO indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,tri_indices.size() * sizeof(int), &(tri_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	std::vector<int> edge_indices;
	convert_quads_to_edges(m_quad_indices,edge_indices);
	m_nb_ind_edges = edge_indices.size();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,m_nb_ind_edges * sizeof(int), &(edge_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}



void MeshQuad::set_matrices(const Mat4& view, const Mat4& projection)
{
	viewMatrix = view;
	projectionMatrix = projection;
}

void MeshQuad::draw(const Vec3& color)
{
	glEnable(GL_CULL_FACE);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 1.0f);

	m_shader_flat->startUseProgram();
	m_shader_flat->sendViewMatrix(viewMatrix);
	m_shader_flat->sendProjectionMatrix(projectionMatrix);
	glUniform3fv(m_shader_flat->idOfColorUniform, 1, glm::value_ptr(color));
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo);
	glDrawElements(GL_TRIANGLES, 3*m_quad_indices.size()/2,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_flat->stopUseProgram();

	glDisable(GL_POLYGON_OFFSET_FILL);

	m_shader_color->startUseProgram();
	m_shader_color->sendViewMatrix(viewMatrix);
	m_shader_color->sendProjectionMatrix(projectionMatrix);
	glUniform3f(m_shader_color->idOfColorUniform, 0.0f,0.0f,0.0f);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo2);
	glDrawElements(GL_LINES, m_nb_ind_edges,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_color->stopUseProgram();
	glDisable(GL_CULL_FACE);
}

