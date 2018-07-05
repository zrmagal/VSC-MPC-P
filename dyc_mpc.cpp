
/*
 *  Autor: Zoé Magalhães (zr.magal@gmail.com)
 *  Mestrando do PPMEC-Unb, matrícula 170172767
 *  Disciplina: Controle Preditivo 01/2018
 *  Professor: André Murilo
 *  
 *   Esta aplicacao realiza uma simulacaco SIL do
 *   MPC aplicado no controle de estabilidade lateral
 *   mediante momento de guinada externo. 
 *   Em que para simular a dinamica veicular esta
 *   sendo utilizada o mesmo modelo linear que
 *   foi utilizado no projeto do controle.
 */


#include <fstream>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <qpOASES.hpp>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

USING_NAMESPACE_QPOASES

/*****************************************************************************
 * Configuracoes
 *****************************************************************************/
//Quantidade estados
#define STATES_SIZE (4)

//Quantidade de parâmetros de decisão da QP
#define QP_SIZE (4)

//Quantidade de entradas controladas
#define COMMAND_SIZE (1)

//Quantidade de entradas não controladas
#define DIST_SIZE (1)

//Quantidade amostras no horizonte de predição
#define HORIZON_SIZE (101)

//Quantidade de estados regulados
#define REG_OUTPUTS_SIZE (1)

//Quantidade de estados com restrição imposta
#define CONST_OUTPUTS_SIZE (1)

//Quantidade total de entradas controladas e não controladas
#define INPUT_SIZE ( DIST_SIZE + COMMAND_SIZE )

//Quantidade de restrições
// - restrição nos estados ( CONST_OUTPUTS_SIZES * HORIZON_SIZE)*2
// - restrição na variação das entradas ( INPUT_SIZE * HORIZON_SIZE)*2
#define CONSTRAINTS_SIZE (( CONST_OUTPUTS_SIZE + INPUT_SIZE )*2*HORIZON_SIZE)

// Tamanho do vetor de trajetoria desejada
#define TRACK_SIZE ( CONST_OUTPUTS_SIZE*HORIZON_SIZE )

// Quantidade de restrição incluindo:
// - restrição no valor das entradas  INPUT_SIZE*HORIZON_SIZE*2
#define BOUNDED_CONSTRAINTS_SIZE ( CONSTRAINTS_SIZE + INPUT_SIZE*HORIZON_SIZE*2 )

/*****************************************************************************
 * Tamanho das matrizes
 *****************************************************************************/
#define H_ROWS QP_SIZE
#define H_COLS QP_SIZE
#define H_SIZE ( H_ROWS*H_COLS )

#define A_ROWS BOUNDED_CONSTRAINTS_SIZE
#define A_COLS QP_SIZE
#define A_SIZE (A_ROWS*A_COLS)

#define PI_E_ROWS (INPUT_SIZE*HORIZON_SIZE)
#define PI_E_COLS QP_SIZE
#define PI_E_SIZE ( PI_E_ROWS*PI_E_COLS )

#define G1_ROWS CONSTRAINTS_SIZE 
#define G1_COLS STATES_SIZE
#define G1_SIZE ( G1_ROWS*G1_COLS )

#define G2_ROWS CONSTRAINTS_SIZE
#define G2_COLS COMMAND_SIZE
#define G2_SIZE ( G2_ROWS*G2_COLS )

#define _G3_SIZE CONSTRAINTS_SIZE

#define F1_ROWS (INPUT_SIZE*HORIZON_SIZE)
#define F1_COLS (STATES_SIZE)
#define F1_SIZE ( F1_ROWS*F1_COLS )

#define F2_ROWS (INPUT_SIZE*HORIZON_SIZE)
#define F2_COLS (REG_OUTPUTS_SIZE*HORIZON_SIZE)
#define F2_SIZE ( F2_ROWS*F2_COLS )

#define F3_ROWS (INPUT_SIZE*HORIZON_SIZE)
#define F3_COLS (INPUT_SIZE)
#define F3_SIZE ( F3_ROWS*F3_COLS )



/*****************************************************************************
 * @brief copia os valores de um arquivo .csv para uma matrix
 * @param[out] arg_matrix é o endereço da matriz que receberá os valores
 * @param[in] arg_rows é a quantidade de linhas da matriz
 * @param[in] arg_cols é a quantidade de colunas da matriz
 * @param[in] arg_csv_name é o nome do arquivo a ser lido
 *
 * @return #TRUE em caso de sucesso, #FALSE em caso de falha
 *****************************************************************************/
static bool get_matrix_from_csv( MatrixXd * arg_matrix, 
                                 unsigned int arg_rows, 
                                 unsigned int arg_cols,
                                 const char * arg_csv_name )
{

    cout << arg_rows << " " << arg_cols << "\r\n";
    arg_matrix->setZero( arg_rows, arg_cols );
    
    std::ifstream file(arg_csv_name);
    for(unsigned int row = 0; row < arg_rows; ++row)
    {
        std::string line;
        std::getline(file, line);
        if ( !file.good() )
            return 0;

        std::stringstream iss(line);

       
        for (unsigned int col = 0; col < arg_cols; ++col)
        {
            std::string val;
            std::getline(iss, val, ',');

            std::stringstream convertor(val);
            convertor >> (*arg_matrix)(row,col);

        }
    }

    return 1;
}

/*****************************************************************************
 * @brief Lê o valor registrado em um arquvo .csv
 * @param[out] arg_value é o endereço da variavel que receberá o valor lido
 * @param[in] arg_csv_name é o nome do arquivo a ser lido
 *
 * @return #TRUE em caso de sucesso, #FALSE em caso de falha
 *****************************************************************************/
static bool get_value_from_csv( real_t * arg_value, 
                                const char * arg_csv_name )
{

    
    std::ifstream file(arg_csv_name);

        std::string line;
        std::getline(file, line);
        if ( !file.good() )
            return 0;

        std::stringstream iss(line);

        std::string val;
        std::getline(iss, val, ',');
       // if ( !iss.good() )
       //     return 0;

        std::stringstream convertor(val);
        convertor >> *arg_value;
    return 1;
}

/*****************************************************************************
 * @brief copia os valores de um arquivo .csv para um vetor
 * @param[out] arg_vector é o endereço do vetor que receberá os valores
 * @param[in] arg_length é a comprimento do vetor
 * @param[in] arg_csv_name é o nome do arquivo a ser lido
 *
 * @return #TRUE em caso de sucesso, #FALSE em caso de falha
 *****************************************************************************/
static bool get_vector_from_csv( VectorXd * arg_vector, 
                                 unsigned int arg_length, 
                                 const char * arg_csv_name )
{

    cout << arg_length << " " << "\r\n";
    arg_vector->setZero( arg_length );
    
    std::ifstream file(arg_csv_name);
    for(unsigned int member = 0; member < arg_length; ++member)
    {
        std::string line;
        std::getline(file, line);
        if ( !file.good() )
            return 0;

        std::stringstream iss(line);

        std::string val;
        std::getline(iss, val, ',');
       // if ( !iss.good() )
       //     return 0;

        std::stringstream convertor(val);
        convertor >> (*arg_vector)(member);

    }

    return 1;
}

/*****************************************************************************
 * @brief descritor do controle preditivo
 *****************************************************************************/
class MPC
{
    MatrixXd  H;                /**< Hessiano da função custo */
    MatrixXd  A;                /**< Matriz das restricoes */
    MatrixXd  PI_E;             /**< Matriz de parametrizacao */
    MatrixXd  G1;               /**< Matriz de restrição dos estados */
    MatrixXd  G2;               /**< Matriz de restrição da variacao do comando*/
    VectorXd  _G3;              /**< Vetor com limiar da restrição */
    MatrixXd  F1;               /**< Gradiente do custo em relacao ao estado atual */
    MatrixXd  F2;               /**< Gradiente do custo em relacao a trajetoria desejada */
    MatrixXd  F3;               /**< Gradiente do custo em relacao ao comando desejado */
    SQProblem *Solver_ptr;      /**< Ponteiro para um descritor do solver qpOASES */
    VectorXd  _last_command;    /**< Último comando enviado */
    real_t  _cmd_ub_;           /**< Limiar superior do comando */
    real_t  _cmd_lb_;           /**< Limiar inferior do comando */
    
    public:
        /**
         * @breif Construtor
         */
        MPC(void); 

        /**
         * @brief Realiza a QP para definir o proximo valor do comando
         * @param[in] arg_steer indica o esterçamento das rodas dianteiras
         * @param[in] arg_desired_yaw_rate indica o valor desejado da taxa
         * da taxa de guinada
         * @param[in] arg_x indica o valor atual dos estaados
         * @param[out] arg_sol informa o valor do comando encontrado como solucao
         */
        void get_next_command( double arg_steer, double arg_desired_yaw_rate, 
                               VectorXd& arg_x, VectorXd& arg_sol );
};

/*****************************************************************************
 * @brief Construtor do descritor de controle preditivo
 *****************************************************************************/
MPC::MPC(void)
{
    int_t _nWSR_ = 200; //Maximo de iteracoes para resolver qp
    VectorXd _B = VectorXd::Zero(BOUNDED_CONSTRAINTS_SIZE); //Limiar das restricoes Ap < _B
    VectorXd _F = VectorXd::Zero(QP_SIZE); //Matriz gradiente da funcao custo J  = p'Hp + _F'p
    //Valores virtuais do estado, comando e trajetoria desejada, para realizar primeira qp
    const VectorXd _x = VectorXd::Zero(STATES_SIZE,1); 
    const VectorXd _u = VectorXd::Zero(COMMAND_SIZE,1);
    const VectorXd _track = VectorXd::Zero(TRACK_SIZE,1);
    VectorXd _sol = VectorXd::Ones(QP_SIZE,1); //Solucao da primeira qp 
    //Restrições impostas a solução da primeira qp
    VectorXd _sol_lb(QP_SIZE);
    VectorXd _sol_ub(QP_SIZE);

    
    /****************************
     *Leitura dos arquivos .csv
     ***************************/
    cout << "\r\nH\r\n";
    if( !get_matrix_from_csv(&H,H_ROWS,H_COLS,"H.csv") )
        cout << "\r\n Erro ao ler H";

    cout << "\r\nA\r\n";
    if( !get_matrix_from_csv(&A,A_ROWS,A_COLS,"A.csv") )
        cout << "\r\n Erro ao ler A";
 
    cout << "\r\nPI_E\r\n";
    if( !get_matrix_from_csv(&PI_E,PI_E_ROWS,PI_E_COLS,"PI_E.csv") )
        cout << "\r\n Erro ao ler PI_E";
    
    cout << "\r\nG1\r\n";
    if( !get_matrix_from_csv(&G1,G1_ROWS,G1_COLS,"G1.csv") )
        cout << "\r\n Erro ao ler G1";

    cout << "\r\nG2\r\n";
    if( !get_matrix_from_csv(&G2,G2_ROWS,G2_COLS,"G2.csv") )
        cout << "\r\n Erro ao ler G2";

    cout << "\r\nG3\r\n";
    if( !get_vector_from_csv(&_G3,_G3_SIZE,"G3.csv") )
        cout << "\r\n Erro ao ler G3";

    cout << "\r\nF1\r\n";
    if( !get_matrix_from_csv(&F1,F1_ROWS,F1_COLS,"F1.csv") )
        cout << "\r\n Erro ao ler F1";

    cout << "\r\nF2\r\n";
    if( !get_matrix_from_csv(&F2,F2_ROWS,F2_COLS,"F2.csv") )
        cout << "\r\n Erro ao ler F2";

    cout << "\r\nF3\r\n";
    if( !get_matrix_from_csv(&F3,F3_ROWS,F3_COLS,"F3.csv") )
        cout << "\r\n Erro ao ler F3";
    
    cout << "\r\ncmd_lb\r\n";
    if( !get_value_from_csv(&_cmd_lb_,"cmd_lb.csv") )
        cout << "\r\n Erro ao ler _cmd_lb_";
    
    cout << "\r\ncmd_ub\r\n";
    if( !get_value_from_csv(&_cmd_ub_,"cmd_ub.csv") )
        cout << "\r\n Erro ao ler cmd_ub";
    

//    cout << "\r\nub\r\n" <<_cmd_ub_;
//    cout << "\r\nlb\r\n" <<_cmd_lb_;

    /****************************
     *Configuracao e start do solver
     ***************************/
    _sol_lb << _cmd_lb_, _cmd_lb_, _cmd_lb_, 0;
    _sol_ub << _cmd_ub_, _cmd_ub_, _cmd_ub_, 0;
    _B.head(CONST_OUTPUTS_SIZE) = _G3.head(CONST_OUTPUTS_SIZE);


    //Configura solver
    Solver_ptr = new qpOASES::SQProblem(QP_SIZE,BOUNDED_CONSTRAINTS_SIZE);
    
    
    //Realiza primeira qp para comecar o solver
    Solver_ptr->init( H.data(), _F.data(), A.data(), _sol_lb.data(), _sol_ub.data(), NULL, _B.data(), _nWSR_, 0, _sol.data());

    //Obtem o resultado da primeira qp
    Solver_ptr->getPrimalSolution(_sol.data());

    //Calcula o valor inicial do comando
    _last_command = PI_E.block(0,0,COMMAND_SIZE,QP_SIZE)*_sol;
}

/*****************************************************************************
 * @brief Fornece o proximo valor do comando
 *****************************************************************************/
void MPC::get_next_command( double arg_steer, double arg_desired_yaw_rate, 
        VectorXd& arg_x,  VectorXd& arg_sol )
{
    VectorXd _B = VectorXd::Zero(BOUNDED_CONSTRAINTS_SIZE);
    VectorXd _F = VectorXd::Zero(QP_SIZE);
    VectorXd _track = VectorXd::Ones(TRACK_SIZE,1)*arg_steer;
    VectorXd _sol = VectorXd::Ones(QP_SIZE,1);
    VectorXd _in_ub(INPUT_SIZE);
    VectorXd _in_lb(INPUT_SIZE);
    VectorXd _horizon_in_ub(INPUT_SIZE*HORIZON_SIZE);
    VectorXd _horizon_in_lb(INPUT_SIZE*HORIZON_SIZE);
    VectorXd _desired_input(INPUT_SIZE);
    VectorXd _sol_lb(QP_SIZE);
    VectorXd _sol_ub(QP_SIZE);
    int_t _nWSR_ = 200;
   

 
    //Comando sejado
    // comando controlado igual a zero e valor atual igual a ele mesmo
    _desired_input << 0, arg_steer; 
    
    //Restricao trivial do comando
    //Comando controlado limitado por seus valores maximos minimos
    //Entrada nao controlada limitada ao seu valor medido a priore
    _in_ub << _cmd_ub_ , arg_steer;
    _in_lb << _cmd_lb_ , arg_steer;
    
    //Expansão da restrição no horzionte de predição
    _horizon_in_ub = _in_ub.replicate(HORIZON_SIZE,1);
    _horizon_in_lb = _in_lb.replicate(HORIZON_SIZE,1);
   
    //Restrição da solução com parametrezição exponencial
    //Parametros do comando controlado limitados aos limites do comando
    //Parametros da entrada nao controlada limitados ao seu valor atual
    _sol_lb << _cmd_lb_, _cmd_lb_, _cmd_lb_, arg_steer;
    _sol_ub << _cmd_ub_, _cmd_ub_, _cmd_ub_, arg_steer;
    

     //Calcular limiar das restrições A * sol < _B
     _B.head(CONSTRAINTS_SIZE) = G1*arg_x + G2*_last_command +_G3;
    _B.segment(CONSTRAINTS_SIZE,INPUT_SIZE*HORIZON_SIZE) = -_horizon_in_lb;
    _B.tail(INPUT_SIZE*HORIZON_SIZE) = _horizon_in_ub;


    //Calcula gradiente da função custo
    _F =  PI_E.transpose()*(F1*arg_x + F2*_track + F3*_desired_input);

    //Realiza qp
    Solver_ptr->hotstart(_F.data(),_sol_lb.data(),_sol_ub.data(),NULL,_B.data(),_nWSR_, 0 );
    
    //Le o resultado da qp
    Solver_ptr->getPrimalSolution(_sol.data());


    cout << "\r\nsol\r\n" << _sol;
    _last_command = PI_E.block(0,0,COMMAND_SIZE,QP_SIZE)*_sol;
    arg_sol = _last_command;
}

int main()
{
   MPC mpc;
   VectorXd _x = VectorXd::Zero(STATES_SIZE);
   VectorXd _sol;

   cout << "init ok\r\n";

   for( int i=0; i< 1000; i++ )
   {
        mpc.get_next_command( 1e-12, 1e-12, _x, _sol );
        cout << "\r\nfinal sol\r\n" << _sol;
   }

}
