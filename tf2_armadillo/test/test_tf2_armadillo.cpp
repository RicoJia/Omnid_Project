/*
 * Copyright (c) Rico Ruotong Jia
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf2_armadillo/tf2_armadillo.h>
#include <gtest/gtest.h>
#include <tf2/convert.h>

typedef arma::Mat<double> mat;
using tf2::fromMsg;
using tf2::toMsg;
using std::endl;

TEST(Tf2ArmadilloTest, geometryMsgsTest1)
{
    geometry_msgs::Transform tf;
    tf.translation.x = 1;
    tf.translation.y = 2;
    tf.translation.z = 3;
    tf.rotation.x = 0;
    tf.rotation.y = 0;
    tf.rotation.z = 0.7071068;
    tf.rotation.w = 0.7071068;
    mat::fixed<4, 4> ret; 
 	fromMsg(tf, ret); 
    mat::fixed<4, 4> expected =
            {{0.0000000, -1.0000000,  0.0000000, 1},
             {1.0000000,  0.0000000,  0.0000000, 2},
             {0.0000000,  0.0000000,  1.0000000, 3},
             {0, 0, 0, 1}};
    EXPECT_TRUE(arma::approx_equal(expected, ret, "absdiff", 1e-3))<<"ret: "<<endl<<ret<<endl<<"expected: "<<expected;
}

TEST(Tf2ArmadilloTest, geometryMsgsTest2){
    geometry_msgs::Transform tf;
    tf.translation.x = 1;
    tf.translation.y = 2;
    tf.translation.z = 3;
    tf.rotation.x = 0.4082483;
    tf.rotation.y = 0.4082483;
    tf.rotation.z = 0.4082483;
    tf.rotation.w = 0.7071068;
    mat::fixed<4, 4> ret; 
 	fromMsg(tf, ret); 
    mat::fixed<4, 4>expected =
            {{0.3333333, -0.2440169,  0.9106836, 1},
             {0.9106836,  0.3333333, -0.2440169, 2},
             {-0.2440169,  0.9106836,  0.3333333, 3},
             {0,0,0,1}};
    EXPECT_TRUE(arma::approx_equal(expected, ret, "absdiff", 1e-3))<<"ret: "<<endl<<ret<<endl<<"expected: "<<expected;
}

TEST(Tf2ArmadilloTest, geometryMsgsTest3){
    geometry_msgs::Transform tf;
    tf.translation.x = 1;
    tf.translation.y = 2;
    tf.translation.z = 3;
    tf.rotation.x = 0.5334021;
    tf.rotation.y = 0.5334021;
    tf.rotation.z = 0.5334021;
    tf.rotation.w = 0.3826834;
    mat::fixed<4, 4> ret; 
 	fromMsg(tf, ret); 
    mat::fixed<4, 4>expected =
            {{-0.1380712, 0.1607873,  0.9772839, 1},
             {0.9772839, -0.1380712,  0.1607873, 2},
             {0.1607873,  0.9772839, -0.1380712, 3},
             {0,0,0,1}};
    EXPECT_TRUE(arma::approx_equal(expected, ret, "absdiff", 1e-3))<<"ret: "<<endl<<ret<<endl<<"expected: "<<expected;
}

TEST(Tf2ArmadilloTest, geometryMsgsTest4){
    geometry_msgs::Transform tf;
    tf.translation.x = 1;
    tf.translation.y = 2;
    tf.translation.z = 3;
    tf.rotation.x = 0.5425318;
    tf.rotation.y = 0.5425318;
    tf.rotation.z = 0.5425318;
    tf.rotation.w = -0.3420201;
    mat::fixed<4, 4> ret; 
 	fromMsg(tf, ret); 
    mat::fixed<4, 4>expected =
            {{-0.1773630, 0.9597951, 0.2175679, 1},
             {0.2175679, -0.1773630,  0.9597951, 2},
             {0.9597951,  0.2175679, -0.1773630, 3},
             {0,0,0,1}};
    EXPECT_TRUE(arma::approx_equal(expected, ret, "absdiff", 1e-3))<<"ret: "<<endl<<ret<<endl<<"expected: "<<endl<<expected;
}

TEST(Tf2ArmadilloTest, tf2TransformTest1)
{
    tf2::Transform tf;
    tf.setOrigin({1,2,3});
    tf.setRotation({0, 0, 0.7071068, 0.7071068});
    mat::fixed<4, 4> ret; 
 	fromMsg(tf, ret); 
    mat::fixed<4, 4> expected =
            {{0.0000000, -1.0000000,  0.0000000, 1},
             {1.0000000,  0.0000000,  0.0000000, 2},
             {0.0000000,  0.0000000,  1.0000000, 3},
             {0, 0, 0, 1}};
    EXPECT_TRUE(arma::approx_equal(expected, ret, "absdiff", 1e-3))<<"ret: "<<endl<<ret<<endl<<"expected: "<<expected;
}

TEST(Tf2ArmadilloTest, tf2TransformTest2){
    tf2::Transform tf;
    tf.setOrigin({1,2,3});
    tf.setRotation({0.4082483, 0.4082483, 0.4082483, 0.7071068});
    mat::fixed<4, 4> ret; 
 	fromMsg(tf, ret); 
    mat::fixed<4, 4>expected =
            {{0.3333333, -0.2440169,  0.9106836, 1},
             {0.9106836,  0.3333333, -0.2440169, 2},
             {-0.2440169,  0.9106836,  0.3333333, 3},
             {0,0,0,1}};
    EXPECT_TRUE(arma::approx_equal(expected, ret, "absdiff", 1e-3))<<"ret: "<<endl<<ret<<endl<<"expected: "<<expected;
}

TEST(Tf2ArmadilloTest, tf2TransformTest3){
    tf2::Transform tf;
    tf.setOrigin({1,2,3});
    tf.setRotation({0.5334021, 0.5334021, 0.5334021, 0.3826834});
    mat::fixed<4, 4> ret; 
 	fromMsg(tf, ret); 
    mat::fixed<4, 4>expected =
            {{-0.1380712, 0.1607873,  0.9772839, 1},
             {0.9772839, -0.1380712,  0.1607873, 2},
             {0.1607873,  0.9772839, -0.1380712, 3},
             {0,0,0,1}};
    EXPECT_TRUE(arma::approx_equal(expected, ret, "absdiff", 1e-3))<<"ret: "<<endl<<ret<<endl<<"expected: "<<expected;
}

TEST(Tf2ArmadilloTest, tf2TransformTest4){
    tf2::Transform tf;
    tf.setOrigin({1,2,3});
    tf.setRotation({0.5425318, 0.5425318, 0.5425318, -0.3420201});
    mat::fixed<4, 4> ret; 
 	fromMsg(tf, ret); 
    mat::fixed<4, 4>expected =
            {{-0.1773630, 0.9597951, 0.2175679, 1},
             {0.2175679, -0.1773630,  0.9597951, 2},
             {0.9597951,  0.2175679, -0.1773630, 3},
             {0,0,0,1}};
    EXPECT_TRUE(arma::approx_equal(expected, ret, "absdiff", 1e-3))<<"ret: "<<endl<<ret<<endl<<"expected: "<<endl<<expected;
}

TEST(Tf2ArmadilloTest, transformTf2Test1)
{
    mat::fixed<4, 4> input =
            {{0.0000000, -1.0000000,  0.0000000, 1},
             {1.0000000,  0.0000000,  0.0000000, 2},
             {0.0000000,  0.0000000,  1.0000000, 3},
             {0, 0, 0, 1}};
    tf2::Transform tf;
    auto output = toMsg(input, tf);
    tf.setOrigin({1,2,3});
    tf.setRotation({0, 0, 0.7071068, 0.7071068});

    EXPECT_NEAR( tf.getRotation().getAngle(), output.getRotation().getAngle(), 1e-3 )<<"expected value: "<< tf.getRotation().getAngle() <<"output: "<< output.getRotation().getAngle();
    auto o_axis = output.getRotation().getAxis();
    auto i_axis =  tf.getRotation().getAxis();
    EXPECT_NEAR(o_axis.getX(),i_axis.getX(), 1e-3)<<" actual: "<<o_axis.getX()<<"expected: "<<i_axis.getX()<<endl;
    EXPECT_NEAR(o_axis.getY(),i_axis.getY(), 1e-3)<<" actual: "<<o_axis.getY()<<"expected: "<<i_axis.getY()<<endl;
    EXPECT_NEAR(o_axis.getZ(),i_axis.getZ(), 1e-3)<<" actual: "<<o_axis.getZ()<<"expected: "<<i_axis.getZ()<<endl;

    auto o_orig = output.getOrigin();
    auto i_orig = tf.getOrigin();
    EXPECT_NEAR( o_orig.getX(), i_orig.getX(), 1e-3 )<<" actual: "<<o_orig.getX()<<"expected: "<<i_orig.getX()<<endl;
    EXPECT_NEAR( o_orig.getY(), i_orig.getY(), 1e-3 )<<" actual: "<<o_orig.getY()<<"expected: "<<i_orig.getY()<<endl;
    EXPECT_NEAR( o_orig.getZ(), i_orig.getZ(), 1e-3 )<<" actual: "<<o_orig.getZ()<<"expected: "<<i_orig.getZ()<<endl;
}

TEST(Tf2ArmadilloTest, transformTf2Test2)
{
    mat::fixed<4, 4>input =
            {{0.3333333, -0.2440169,  0.9106836, 1},
             {0.9106836,  0.3333333, -0.2440169, 2},
             {-0.2440169,  0.9106836,  0.3333333, 3},
             {0,0,0,1}};
    tf2::Transform tf;
    auto output = toMsg(input, tf);
    tf.setOrigin({1,2,3});
    tf.setRotation({0.4082483, 0.4082483, 0.4082483, 0.7071068});

    EXPECT_NEAR( tf.getRotation().getAngle(), output.getRotation().getAngle(), 1e-3 )<<"expected value: "<< tf.getRotation().getAngle() <<"output: "<< output.getRotation().getAngle();
    auto o_axis = output.getRotation().getAxis();
    auto i_axis =  tf.getRotation().getAxis();
    EXPECT_NEAR(o_axis.getX(),i_axis.getX(), 1e-3)<<" actual: "<<o_axis.getX()<<"expected: "<<i_axis.getX()<<endl;
    EXPECT_NEAR(o_axis.getY(),i_axis.getY(), 1e-3)<<" actual: "<<o_axis.getY()<<"expected: "<<i_axis.getY()<<endl;
    EXPECT_NEAR(o_axis.getZ(),i_axis.getZ(), 1e-3)<<" actual: "<<o_axis.getZ()<<"expected: "<<i_axis.getZ()<<endl;

    auto o_orig = output.getOrigin();
    auto i_orig = tf.getOrigin();
    EXPECT_NEAR( o_orig.getX(), i_orig.getX(), 1e-3 )<<" actual: "<<o_orig.getX()<<"expected: "<<i_orig.getX()<<endl;
    EXPECT_NEAR( o_orig.getY(), i_orig.getY(), 1e-3 )<<" actual: "<<o_orig.getY()<<"expected: "<<i_orig.getY()<<endl;
    EXPECT_NEAR( o_orig.getZ(), i_orig.getZ(), 1e-3 )<<" actual: "<<o_orig.getZ()<<"expected: "<<i_orig.getZ()<<endl;
}

TEST(Tf2ArmadilloTest, transformTf2Test3)
{
    mat::fixed<4, 4>input =
            {{-0.1380712, 0.1607873,  0.9772839, 1},
             {0.9772839, -0.1380712,  0.1607873, 2},
             {0.1607873,  0.9772839, -0.1380712, 3},
             {0,0,0,1}};
    tf2::Transform tf;
    auto output = toMsg(input, tf);
    tf.setOrigin({1,2,3});
    tf.setRotation({0.5334021, 0.5334021, 0.5334021, 0.3826834});

    EXPECT_NEAR( tf.getRotation().getAngle(), output.getRotation().getAngle(), 1e-3 )<<"expected value: "<< tf.getRotation().getAngle() <<"output: "<< output.getRotation().getAngle();
    auto o_axis = output.getRotation().getAxis();
    auto i_axis =  tf.getRotation().getAxis();
    EXPECT_NEAR(o_axis.getX(),i_axis.getX(), 1e-3)<<" actual: "<<o_axis.getX()<<"expected: "<<i_axis.getX()<<endl;
    EXPECT_NEAR(o_axis.getY(),i_axis.getY(), 1e-3)<<" actual: "<<o_axis.getY()<<"expected: "<<i_axis.getY()<<endl;
    EXPECT_NEAR(o_axis.getZ(),i_axis.getZ(), 1e-3)<<" actual: "<<o_axis.getZ()<<"expected: "<<i_axis.getZ()<<endl;

    auto o_orig = output.getOrigin();
    auto i_orig = tf.getOrigin();
    EXPECT_NEAR( o_orig.getX(), i_orig.getX(), 1e-3 )<<" actual: "<<o_orig.getX()<<"expected: "<<i_orig.getX()<<endl;
    EXPECT_NEAR( o_orig.getY(), i_orig.getY(), 1e-3 )<<" actual: "<<o_orig.getY()<<"expected: "<<i_orig.getY()<<endl;
    EXPECT_NEAR( o_orig.getZ(), i_orig.getZ(), 1e-3 )<<" actual: "<<o_orig.getZ()<<"expected: "<<i_orig.getZ()<<endl;
}

TEST(Tf2ArmadilloTest, transformTf2Test4)
{
    mat::fixed<4, 4>input =
            {{-0.1773630, 0.9597951, 0.2175679, 1},
             {0.2175679, -0.1773630,  0.9597951, 2},
             {0.9597951,  0.2175679, -0.1773630, 3},
             {0,0,0,1}};
    tf2::Transform tf;
    auto output = toMsg(input, tf);
    tf.setOrigin({1,2,3});
    tf.setRotation({0.5425318, 0.5425318, 0.5425318, -0.3420201});

    EXPECT_NEAR( tf.getRotation().getAngle(), output.getRotation().getAngle(), 1e-3 )<<"expected value: "<< tf.getRotation().getAngle() <<"output: "<< output.getRotation().getAngle();
    auto o_axis = output.getRotation().getAxis();
    auto i_axis =  tf.getRotation().getAxis();
    EXPECT_NEAR(o_axis.getX(),i_axis.getX(), 1e-3)<<" actual: "<<o_axis.getX()<<"expected: "<<i_axis.getX()<<endl;
    EXPECT_NEAR(o_axis.getY(),i_axis.getY(), 1e-3)<<" actual: "<<o_axis.getY()<<"expected: "<<i_axis.getY()<<endl;
    EXPECT_NEAR(o_axis.getZ(),i_axis.getZ(), 1e-3)<<" actual: "<<o_axis.getZ()<<"expected: "<<i_axis.getZ()<<endl;

    auto o_orig = output.getOrigin();
    auto i_orig = tf.getOrigin();
    EXPECT_NEAR( o_orig.getX(), i_orig.getX(), 1e-3 )<<" actual: "<<o_orig.getX()<<"expected: "<<i_orig.getX()<<endl;
    EXPECT_NEAR( o_orig.getY(), i_orig.getY(), 1e-3 )<<" actual: "<<o_orig.getY()<<"expected: "<<i_orig.getY()<<endl;
    EXPECT_NEAR( o_orig.getZ(), i_orig.getZ(), 1e-3 )<<" actual: "<<o_orig.getZ()<<"expected: "<<i_orig.getZ()<<endl;
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}