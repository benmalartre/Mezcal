// std
#include <iostream>
#include <stdlib.h>
#include <vector>

// threads
#include "tinycthread/tinycthread.h"

// openGL
#include "gl3w/GL/gl3w.h"
#include "glfw/glfw3.h"

// 3D framework
#include "framework/vector3.h"
#include "framework/mesh.h"
#include "framework/camera.h"
#include "framework/random.h"
#include "framework/shader.h"

// bullet
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"


using namespace BOB;
using namespace std;

//keep the collision shapes, for deletion/cleanup
btAlignedObjectArray<btCollisionShape*>	collisionShapes;
btBroadphaseInterface*	broadphase;
btCollisionDispatcher*	dispatcher;
btConstraintSolver*	solver;
btDefaultCollisionConfiguration* collisionConfiguration;
btDiscreteDynamicsWorld* dynamicsWorld;


Matrix4 M;

Mesh* createBall()
{
    return new Mesh();
}

Mesh* createSquare()
{
    unsigned nbVertices = 4;
    unsigned nbSamples = 5;
    std::vector<Vector3> positions;
    positions.resize(4);
    
    positions[0].x = -0.5f;
    positions[0].y = 0.5f;
    positions[0].z = 0.0f;
    
    positions[1].x = 0.5f;
    positions[1].y = 0.5f;
    positions[1].z = 0.0f;
    
    positions[2].x = 0.5f;
    positions[2].y = -0.5f;
    positions[2].z = 0.0f;
    
    positions[3].x = -0.5f;
    positions[3].y = -0.5f;
    positions[3].z = 0.0f;
    
    std::vector<signed> description;
    description.resize(5);
    description[0] = 0;
    description[1] = 1;
    description[2] = 2;
    description[3] = 3;
    description[4] = -2;
    
    Mesh* mesh = new Mesh(nbVertices,(float*)&positions[0].x, nbSamples, (signed*)&description[0]);
    
    return mesh;
}

void openGLInfos()
{
    printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION),
    glGetString(GL_SHADING_LANGUAGE_VERSION));
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if(action == GLFW_PRESS || action == GLFW_REPEAT )
    {
        switch(key)
        {
            case GLFW_KEY_SPACE:
                cout << "SPACE BAR EVENT" << endl;
                M[13] += 0.05f;
                break;
            case GLFW_KEY_LEFT:
                cout << "LEFT ARROW EVENT" << endl;
                M[12] += 0.05f;
                break;
            case GLFW_KEY_RIGHT:
            	cout << "RIGHT ARROW EVENT" << endl;
                M[13] += 0.05f;
                break;
        }
    }

    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
        cout << "SPACE BAR PRESSED" << endl;
    else if(key == GLFW_KEY_SPACE && action == GLFW_RELEASE)
    	cout << "SPACE BAR RELEASE" << endl;
}

static void refresh_callback(GLFWwindow *window)
{
    glClearColor(random_0_1(), random_0_1(), random_0_1(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glfwSwapBuffers(window);
}

static void resize_callback(GLFWwindow *window, int width, int height)
{
    width = MINIMUM(width, 1);
    height = MINIMUM(height, 1);
    glViewport(0, 0, width, height);
    glClearDepth(1.0);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glEnable(GL_DEPTH_TEST);
}

//void mouse_callback(GLFWindow* window,)

void createDynamicWorld()
{
    ///collision configuration contains default setup for memory, collision setup
    collisionConfiguration = new btDefaultCollisionConfiguration();
    //m_collisionConfiguration->setConvexConvexMultipointIterations();
    
    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new	btCollisionDispatcher(collisionConfiguration);
    
    broadphase = (btBroadphaseInterface*)new btDbvtBroadphase();
    
    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
    solver = (btConstraintSolver*)sol;
    
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, -10, 0));

}

void GLCheckError(std::string msg)
{
    unsigned err = glGetError();
    if(err)
    {
        switch(err)
        {
            case GL_INVALID_ENUM:
                std::cout << "[OpenGL Error] INVALID ENUM :" << msg << std::endl;
                break;
            case GL_INVALID_VALUE:
                std::cout << "[OpenGL Error] INVALID VALUE :" << msg << std::endl;
                break;
            case GL_INVALID_OPERATION:
                std::cout << "[OpenGL Error] INVALID OPERATION :" << msg << std::endl;
                break;
            case GL_OUT_OF_MEMORY:
                std::cout << "[OpenGL Error] OUT OF MEMORY :" << msg << std::endl;
                break;
            case GL_STACK_UNDERFLOW:
                std::cout << "[OpenGL Error] STACK UNDERFLOW :" << msg << std::endl;
                break;
            case GL_STACK_OVERFLOW:
                std::cout << "[OpenGL Error] STACK OVERFLOW :" << msg << std::endl;
                break;
            default:
                std::cout << "[OpenGL Error] UNKNOWN ERROR :" << msg << std::endl;
                break;
        }
    }
}

static int thread_test_args (void * aArg)
{
    return *(int*)aArg;
}

#define TEST_THREAD_ARGS_N_THREADS 4
static void test_thread_arg_and_retval(void)
{
    thrd_t threads[TEST_THREAD_ARGS_N_THREADS];
    int ids[TEST_THREAD_ARGS_N_THREADS];
    int retval;
    int i;
    
    for (i = 0; i < TEST_THREAD_ARGS_N_THREADS; i++)
    {
        ids[i] = rand();
        thrd_create(&(threads[i]), thread_test_args, (void*) &(ids[i]));
    }
    
    for (i = 0; i < TEST_THREAD_ARGS_N_THREADS; i++)
    {
        thrd_join(threads[i], &retval);
        std::cout << "Thread " << i << " ---> ID : " << (retval == ids[i])<<std::endl;
    }
    
    std::cout << "Threads Finshed!!!" << std::endl;
}

void buildGLData(Mesh* mesh)
{
    /*
     ;Get Underlying Geometry
     Protected *geom.Geometry::PolymeshGeometry_t = *p\geom
     Protected nbs = *geom\nbsamples
     If nbs <3 : ProcedureReturn : EndIf
     
     Protected GLfloat_s.GLfloat
     
     ; Get Polymesh Datas
     Protected s3 = SizeOf(GLfloat_s) * 3
     Protected s4 = SizeOf(GLfloat_s) * 4
     Protected size_p.i = nbs * s3
     Protected size_c.i = nbs * s4
     Protected size_t.i = 4*size_p + size_c
     
     ; Allocate Memory
     Protected *flatdata = AllocateMemory(size_p)
     Protected i
     Protected *v.v3f32
     Protected *c.c4f32
     
     ; Push Buffer to GPU
     glBufferData(#GL_ARRAY_BUFFER,size_t,#Null,#GL_STATIC_DRAW)
     
     ;     Protected startT1.d = Time::Get()
     ; POSITIONS
     ;-------------------------------------------------------------
     Protected size_v = CArray::GetItemSize(*geom\a_positions)
     Protected size_i = CArray::GetItemSize(*geom\a_triangleindices)
     Protected id.l
     For i=0 To nbs-1
     id = PeekL(*geom\a_triangleindices\data+(i*size_i))
     *v = *geom\a_positions\data + size_v*id
     CopyMemory(*v,*flatdata+i*s3,s3)
     Next i
     
     ;     FreeMemory(*flatdata)
     ;     Protected endT1.d = Time::get()
     ;
     ;     Protected startT2.d = Time::Get()
     ;     ; POSITIONS
     ;     ;-------------------------------------------------------------
     ;     For i=0 To nbs-1
     ;       *v = CArray::GetValue(*geom\a_positions,CArray::GetValueL(*geom\a_triangleindices,i))
     ;       CopyMemory(*v,*flatdata+i*s3,s3)
     ;     Next i
     ;
     ;     Protected endT2.d = Time::get()
     
     glBufferSubData(#GL_ARRAY_BUFFER,0,size_p,*flatdata)
     FreeMemory(*flatdata)
     
     ;     Protected delta1.d = endT1-startT1
     ;     Protected delta2.d = endT2-startT2
     ;     MessageRequester("Time Difference",StrF(delta1)+Chr(10)+StrF(delta2))
     
     ; NORMALS
     ;-------------------------------------------------------------
     glBufferSubData(#GL_ARRAY_BUFFER,size_p,size_p,CArray::GetPtr(*geom\a_normals,0))
     
     ; TANGENTS
     ;-------------------------------------------------------------
     glBufferSubData(#GL_ARRAY_BUFFER,2*size_p,size_p,CArray::GetPtr(*geom\a_tangents,0))
     
     ; UVWS
     ;-------------------------------------------------------------
     glBufferSubData(#GL_ARRAY_BUFFER,3*size_p,size_p,CArray::GetPtr(*geom\a_uvws,0))
     
     ; COLORS
     ;-------------------------------------------------------------
     Protected c
     Protected *cl.c4f32
     
     glBufferSubData(#GL_ARRAY_BUFFER,4*size_p,size_c,CArray::GetPtr(*geom\a_colors,0))
     
     ; Attibute Position 0
     glEnableVertexAttribArray(0)
     glVertexAttribPointer(0,3,#GL_FLOAT,#GL_FALSE,0,0)
     
     ;Attibute Normal 1
     glEnableVertexAttribArray(1)
     glVertexAttribPointer(1,3,#GL_FLOAT,#GL_FALSE,0,size_p)
     
     ;Attibute Tangent 2
     glEnableVertexAttribArray(2)
     glVertexAttribPointer(2,3,#GL_FLOAT,#GL_FALSE,0,2*size_p)
     
     ;Attibute UVWs 2
     glEnableVertexAttribArray(3)
     glVertexAttribPointer(3,3,#GL_FLOAT,#GL_FALSE,0,3*size_p)
     
     ; Attribute Color 3
     glEnableVertexAttribArray(4)
     glVertexAttribPointer(4,4,#GL_FLOAT,#GL_FALSE,0,4*size_p)
     
     EndProcedure
     */
}



int main(void)
{
    test_thread_arg_and_retval();
    GLFWwindow* window;
    Mesh* mesh = createSquare();
    M.identity();
    
    Camera camera;
    
    Vector3 pos(0.5,0.5,0.5);
    Vector3 lookat(0,0,0);
    Vector3 up(0,1,0);
    camera.setPosition(pos);
    camera.setLookAt(lookat);
    camera.setUp(up);
    float ratio = 640.f/480.f;
    camera.setDescription(66.f, ratio, 0.0001f, 100000.f);
    camera.updateProjection();
    
    Matrix4 identity;
    identity.identity();
    //glUseProgram(0);
    //GLuint vbo;
    
    
    createDynamicWorld();
    
    /* Initialize the library */
    if (!glfwInit())
        return -1;
    
    /* Create a windowed mode window and its OpenGL context */
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    window = glfwCreateWindow(640, 480, "QA", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    
    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    
    // attach callbacks
    glfwSetKeyCallback(window, key_callback);
    glfwSetFramebufferSizeCallback(window, resize_callback);
    glfwSetWindowRefreshCallback(window, refresh_callback);
    
    if (gl3wInit()) {
        fprintf(stderr, "failed to initialize OpenGL\n");
        return -1;
    }
    if (!gl3wIsSupported(3, 2)) {
        fprintf(stderr, "OpenGL 3.2 not supported\n");
        return -1;
    }
    
    // create shader
    GLSLProgram pgm("simple");
    glUseProgram(pgm.get());
    GLCheckError("USE SIMPLE GLSL PROGRAM : ");
    
    glViewport(0, 0, 640, 480);
    
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    
    GLuint vbo;
    glGenBuffers(1,&vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    size_t s = 1024;
    glBufferData(GL_ARRAY_BUFFER,s,NULL,GL_STATIC_DRAW);
    
    
    

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        
        /* randomize color */
        float r = (float) (rand()) / (float) (RAND_MAX);
        float g = (float) (rand()) / (float) (RAND_MAX);
        float b = (float) (rand()) / (float) (RAND_MAX);
        
        /* randomize camera */
        pos.x = (float) (rand()) / (float) (RAND_MAX)*12;
        pos.y = (float) (rand()) / (float) (RAND_MAX)*4;
        pos.z = (float) (rand()) / (float) (RAND_MAX)*12;
        camera.setPosition(pos);
        
        //float fov = ((float) (rand()) / (float) (RAND_MAX))*60.0f;
        //camera.setDescription(fov, ratio, 0.0001f, 100000.f);

        //camera.lookAt();
        camera.updateProjection();

        //glMatrix
        
        glClearColor(r,g,b,0.0f);
        
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);
        /*
        glMatrixMode(GL_PROJECTION);
        Matrix4 tp = camera.projectionMatrix();
        glLoadMatrixf(&tp.v[0]);
    	
        glMatrixMode(GL_MODELVIEW);
        Matrix4 tv = camera.viewMatrix();
        glLoadMatrixf(&tv.v[0]);
        
        // Draw a triangle:
        glBegin(GL_TRIANGLES);
        
        // draw square
        for(unsigned i=0; i<mesh->nbTriangles*3; i++)
        {
            glVertex3f(mesh->positions[mesh->triangles[i]].x,
                       mesh->positions[mesh->triangles[i]].y,
                       mesh->positions[mesh->triangles[i]].z);
        }
        glEnd();
         */
        /* Swap front and back buffers */
        glfwSwapBuffers(window);
        
        /* Poll for and process events */
        glfwPollEvents();
    }
    delete mesh;
    
    glfwTerminate();
    return 0;
}
