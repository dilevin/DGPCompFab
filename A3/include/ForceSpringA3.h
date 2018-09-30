//
//  ForceSpring.h
//  Gauss
//
//  Created by David Levin on 3/4/17.
//
//

#ifndef ForceSpring_h
#define ForceSpring_h

#include <ForceExternal.h>
#include <DOFParticle.h>
#include <UtilitiesEigen.h>
#include <unsupported/Eigen/AutoDiff>

namespace Gauss {
    namespace ParticleSystem {
        
        template<typename DataType, typename Position1, typename Position2>
        class ForceSpringImpl
        {
        public:
            
            ForceSpringImpl(Position1 q0, Position2 q1, double l0, double k) : m_q0(0), m_q1(0) {
                m_q0 = q0;
                m_q1 = q1;
                m_l0 = l0;
                m_k = k;
            }
            
            //Compute the energy of a single spring
            inline DataType getEnergy(State<DataType> &state) {
                
                double energy = 0.0;
                //---- YOUR CODE HERE ----//
               
                //---- END YOUR CODE HERE ----//
                return energy;
            }
            
            //forces always act on at least one DOF of the system this function returns which DOF the are acting on.
            inline auto & getDOF(unsigned int index) {
                
                if(index == 0) {
                    return *m_q0.getDOF();
                } else {
                    return *m_q1.getDOF();
                }
            }
            
            inline auto & getPosition0() {
                    return m_q0;
            }

            inline void setPosition0(Position1 q0) {
                    m_q0 = q0;
            }
            
            inline auto & getPosition1() {
                return m_q1;
            }

            inline void setPosition1(Position2 q1) {
                    m_q1 = q1;
            }
            
            inline unsigned int getNumDOF() { return 2; }
            
            const DataType & getStiffness() const { return m_k; }
            void setStiffness(const DataType &k) { m_k = k; }

            const DataType & getRestLength() const { return m_l0; }
            void setRestLength(const DataType &l0) { m_l0 = l0; }

            //Compute the forces acting on the end points of a spring.
            template <typename Vector>
            inline void getForce(Vector &f, State<DataType> &state) {
                
                //x0 (x1) is the three dimensional position of the first (second) node of a single spring.
                //m_l0 is the length of the spring in its undeformed state
                auto x0 = m_q0(state);
                auto x1 = m_q1(state);
                Eigen::Vector3d f0, f1; //force acting on first (second) node of spring
                
                double mag = (x1-x0).norm(); //current length of spring
                
                if(fabs(mag) < 1e-8) {
                    mag = 1e-8;
                }
                
                //---- YOUR CODE HERE ----//
                f0.setZero();
                f1.setZero();
                //---- END YOUR CODE HERE ----//
                
                assign(f, f0, std::array<DOFBase<DataType,0> , 1>{{*m_q0.getDOF()}});
                assign(f, f1, std::array<DOFBase<DataType,0> , 1>{{*m_q1.getDOF()}});
            }
            
            
            //Compute the 6x6 stiffness matrix for a spring.
            template <typename Matrix>
            inline void getStiffnessMatrix(Matrix &H, State<DataType> &state) {
                
                //x0 (x1) is the three dimensional position of the first (second) node of a single spring.
                //m_l0 is the length of the spring in its undeformed state
                auto x0 = m_q0(state);
                auto x1 = m_q1(state);
                
                //stiffness matrix
                Eigen::Matrix<double, 6,6> Hspring;
                
                double mag = (x1-x0).norm(); //current length of spring
                
                if(fabs(mag) < 1e-8) {
                    mag = 1e-8;
                }
                
                //---- YOUR CODE HERE ----//
                Hspring.setZero();
                //---- END YOUR CODE HERE ----//
                
                assign(H, Hspring, std::array<DOFBase<DataType,0> , 1>{{*m_q0.getDOF()}}, std::array<DOFBase<DataType,0> , 1>{{*m_q1.getDOF()}});
                
            }
            
        protected:
            
            DataType m_l0, m_k; //spring original length and stiffness
            Position1 m_q0; //spring end points
            Position2 m_q1;
            
        private:
        };
        
        template<typename DataType, typename Position1, typename Position2>
        using ForceSpring = Force<DataType, ForceSpringImpl<DataType, Position1, Position2> >;
        
        template<typename DataType>
        using PosParticle = PositionEigen<DataType, DOFParticle<DataType> >;
        
        template<typename DataType>
        using ForceSpringParticles = Force<DataType, ForceSpringImpl<DataType,
                                                           PosParticle<DataType>,
                                                           PosParticle<DataType> > >;
    }
}

#endif /* ForceSpring_h */
