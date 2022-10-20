/*
********************************************************************************
MIT License

Copyright(c) 2018 Christopher Brandt

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
********************************************************************************
*/

/* libIGL include files for loading meshes and the OpenGL viewer */
#include "igl/readOBJ.h"
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/glfw/ViewerPlugin.h"

/* HRPD include files */
#include "ProjDynTypeDef.h"
#include "ProjDynSimulator.h"

/* Constructs a simulator object, sets constraints, adds gravity, a floor with friction and
triggers the precomputation. */
ProjDynSimulator* initSimulator(PD::PDPositions verts, PD::PDTriangles faces, PD::PDPositions velos, std::string meshURL) {
	// Construct simulator object from this mesh and provide initial velcocities,
	// as well as a few reduction and simulation parameters, which are explained
	// in the _README.txt
	double timeStep = 16;
	int numberSamplesForVertexPosSubspace = 150; // The number of degrees of freedom for the mesh vertex positions will be 12 times that
	double radiusMultiplierForVertexPosSubspace = 1.1; // The larger this number, the larger the support of the base functions.
	int dimensionOfConstraintProjectionsSubspace = 120; // The constraint projections subspace will be constructed to be twice that size and then condensed via an SVD
	double radiusMultiplierForConstraintProjectionsSubspace = 2.2;
	int numberSampledConstraints = 1000; // Number of constraints that will be evaluated each iteration
	ProjDynSimulator* sim =
		new ProjDynSimulator(faces, verts, velos, timeStep, 
			numberSamplesForVertexPosSubspace, radiusMultiplierForVertexPosSubspace,
			dimensionOfConstraintProjectionsSubspace, radiusMultiplierForConstraintProjectionsSubspace,
			numberSampledConstraints, 
			2, 0, true, meshURL,
			0., 3);

	// For the simulation to be meaningful we need some sorts of constraints
	// The following method adds volume preservation constraints to all tets
	sim->addTetStrain(0.00051, 1.f, 1.f);

	// We add gravity and floor friction+repulsion to the simulation
	sim->addGravity(0.0001);
	sim->addFloor(1, -2, 1);
	sim->setFrictionCoefficient(0.5, 0.05);

	// The call to the setup function computes the subspaces for vertex
	// positions and constraint projections and factorizes all linear systems
	// for local and global steps
	sim->setup();

	return sim;
}

/*
This class is an auxilary class to enable pre-draw and key-press events with
the libIGL OpenGL viewer.
Skip to the main() function below first to get a basic idea of this example.
*/
class SimViewer : public igl::opengl::glfw::ViewerPlugin {

public:

	ProjDynSimulator* m_sim = nullptr;
	PDPositions& m_verts;
	PDPositions& m_velos;
	PDPositions m_gripPos;
	PDTriangles& m_faces;
	std::string m_url;
	StopWatch m_simTimer;
	std::vector<unsigned int> m_gripVerts;
	bool m_isGripping = false;
	bool m_toggleStiff = false;
	bool m_toggleSlowmo = false;
	int m_numIterations;
	int m_numVertices;

	SimViewer(PDPositions& verts, PDTriangles& faces, PDPositions& velos, std::string url, int numIterations) :
		m_verts(verts),
		m_faces(faces),
		m_velos(velos),
		m_simTimer(10000, 100000),
		m_url(url),
		m_isGripping(false),
		m_toggleStiff(false)
	{
		m_sim = initSimulator(m_verts, m_faces, m_velos, m_url);
		m_numIterations = numIterations;
		m_numVertices = m_verts.rows();

		/* The following is specific to the armadillo.obj mesh, it's a list of vertices on the
		   back, on which the mesh is "hung up" when the user presses 3.
		   Change accordingly if using other meshes (can also be an emtpy list). */
		std::vector<unsigned int> backVerts { 2501, 2567, 2609, 2619, 2620, 2637, 2666, 2678, 2732, 2735, 2757, 2761, 2788, 2812, 2842, 2850, 2871, 2899, 2904, 2913, 2914, 2918, 2961, 2967, 2975, 2995, 3009, 3033, 3041, 3050, 3052, 3067, 3081, 3089, 3138, 3160, 3172, 3185, 3194, 3206, 3232, 3245, 3270, 3295, 3301, 3325, 3330, 3364, 3366, 3391, 3427, 3459, 3472, 3499, 3518, 3542, 3546, 3547, 3568, 3578, 3596, 3609, 3616, 3626, 3630, 3656, 3667, 3683, 3691, 3695, 3697, 3700, 3701, 3705, 3711, 3714, 3716, 3735, 3736, 3746, 3749, 3750, 3751, 3752, 3763, 3764, 3769, 3770, 3771, 3774, 3775, 3782, 3785, 3790, 3791, 3794, 3795, 3804, 3805, 3806, 3810, 3812, 3816, 3819, 3823, 3827, 3828, 3836, 3839, 3841, 3843, 3845, 3846, 3853, 3858, 3859, 3863, 3864, 3867, 3868, 3871, 3875, 3876, 3877, 3878, 3880, 3886, 3891, 3894, 3895, 3896, 3901, 3904, 3907, 3908, 3912, 3913, 3914, 3922, 3923, 3928, 3936, 3943, 3951, 3954, 3955, 3956, 3957, 3959, 3965, 3969, 3971, 3977, 3980, 3982, 3995, 3999, 4000, 4001, 4003, 4006, 4018, 4032, 4041, 4043, 4053, 4068, 4072, 4075, 4080, 4081, 4083, 4086, 4091, 4092, 4094, 4095, 4096, 4100, 4101, 4103, 4107, 4113, 4119, 4122, 4124, 4130, 4132, 4134, 4141, 4143, 4147, 4150, 4152, 4155, 4162, 4165, 4166, 4167, 4169, 4170, 4177, 4186, 4190, 4193, 4194, 4205, 4210, 4214, 4228, 4230, 4232, 4241, 4243, 4255, 4257, 4258, 4269, 4271, 4276, 4278, 4284, 4285, 4288, 4289, 4302, 4306, 4309, 4312, 4319, 4323, 4326, 4331, 4339, 4346, 4352, 4353, 4360, 4361, 4362, 4363, 4382, 4387, 4391, 4395, 4396, 4400, 4403, 4407, 4412, 4420, 4426, 4429, 4443, 4444, 4447, 4450, 4452, 4453, 4454, 4456, 4459, 4462, 4469, 4476, 4477, 4479, 4480, 4486, 4489, 4490, 4496, 4499, 4502, 4504, 4510, 4511, 4512, 4516, 4517, 4520, 4521, 4522, 4523, 4524, 4525, 4526, 4527, 4528, 4531, 4541, 4543, 4550, 4551, 4552, 4553, 4554, 4559, 4560, 4564, 4567, 4569, 4570, 4571, 4574, 4586, 4588, 4590, 4591, 4592, 4597, 4599, 4612, 4613, 4620, 4631, 4654, 4655, 4658, 4661, 4662, 4667, 4675, 4686, 4689, 4695, 4699, 4700, 4702, 4705, 4706, 4709, 4715, 4720, 4734, 4740, 4741, 4746, 4748, 4754, 4755, 4760, 4764, 4777, 4779, 4788, 4794, 4797, 4814, 4816, 4823, 4824, 4829, 4830, 4832, 4839, 4845, 4847, 4856, 4861, 4863, 4864, 4865, 4874, 4875, 4876, 4880, 4885, 4886, 4901, 4902, 4912, 4913, 4920, 4921, 4926, 4927, 4932, 4938, 4943, 4948, 4954, 4956, 4957, 4958, 4963, 4965, 4974, 4977, 4980, 4983, 4988, 4994, 4996, 4998, 5003, 5004, 5006, 5007, 5021, 5022, 5033, 5034, 5036, 5039, 5041, 5050, 5051, 5052, 5054, 5056, 5057, 5061, 5071, 5072, 5074, 5078, 5085, 5086, 5088, 5089, 5091, 5094, 5096, 5097, 5101, 5102, 5106, 5111, 5112, 5116, 5118, 5120, 5123, 5125, 5129, 5136, 5137, 5141, 5142, 5146, 5147, 5148, 5152, 5155, 5156, 5158, 5163, 5166, 5169, 5173, 5178, 5179, 5180, 5181, 5187, 5190, 5192, 5193, 5197, 5203, 5205, 5206, 5213, 5217, 5218, 5220, 5221, 5230, 5231, 5233, 5236, 5239, 5246, 5249, 5252, 5254, 5255, 5256, 5265, 5268, 5281, 5282, 5284, 5290, 5295, 5297, 5301, 5302, 5304, 5306, 5307, 5311, 5312, 5316, 5322, 5323, 5324, 5328, 5330, 5333, 5336, 5342, 5343, 5348, 5351, 5354, 5355, 5362, 5363, 5366, 5370, 5372, 5375, 5378, 5379, 5380, 5385, 5388, 5396, 5402, 5409, 5411, 5415, 5419, 5420, 5421, 5423, 5428, 5429, 5432, 5442, 5443, 5449, 5455, 5456, 5464, 5467, 5474, 5475, 5482, 5486, 5487, 5490, 5491, 5493, 5494, 5499, 5511, 5518, 5522, 5523, 5524, 5525, 5540, 5542, 5543, 5544, 5545, 5557, 5566, 5567, 5568, 5569, 5572, 5576, 5580, 5581, 5582, 5587, 5594, 5597, 5601, 5605, 5609, 5610, 5612, 5615, 5619, 5626, 5628, 5632, 5634, 5636, 5639, 5641, 5642, 5648, 5652, 5653, 5655, 5663, 5665, 5675, 5676, 5686, 5689, 5704, 5706, 5709, 5714, 5715, 5726, 5733, 5740, 5741, 5750, 5751, 5752, 5754, 5755, 5758, 5761, 5763, 5764, 5765, 5771, 5776, 5781, 5784, 5789, 5792, 5793, 5800, 5802, 5804, 5806, 5808, 5809, 5816, 5818, 5822, 5831, 5834, 5835, 5840, 5843, 5844, 5850, 5851, 5853, 5854, 5855, 5857, 5861, 5862, 5863, 5867, 5882, 5885, 5886, 5891, 5898, 5901, 5904, 5905, 5911, 5917, 5923, 5931, 5932, 5937, 5939, 5950, 5951, 5970, 5971, 5973, 5975, 5977, 5979, 5982, 5985, 5986, 5988, 5990, 5995, 6003, 6006, 6009, 6024, 6026, 6031, 6034, 6038, 6039, 6046, 6058, 6059, 6060, 6062, 6067, 6068, 6073, 6085, 6092, 6106, 6115, 6117, 6126, 6128, 6131, 6141, 6165, 6166, 6167, 6169, 6172, 6175, 6176, 6179, 6184, 6190, 6195, 6197, 6199, 6210, 6216, 6222, 6224, 6228, 6233, 6238, 6251, 6265, 6268, 6272, 6279, 6293, 6297, 6307, 6311, 6312, 6315, 6321, 6323, 6327, 6329, 6332, 6334, 6345, 6350, 6356, 6357, 6362, 6364, 6375, 6384, 6386, 6389, 6400, 6404, 6407, 6409, 6428, 6435, 6439, 6440, 6451, 6452, 6455, 6456, 6457, 6460, 6465, 6466, 6467, 6471, 6476, 6481, 6484, 6491, 6494, 6496, 6499, 6500, 6501, 6505, 6512, 6516, 6523, 6524, 6541, 6552, 6554, 6559, 6563, 6567, 6570, 6572, 6579, 6580, 6583, 6587, 6590, 6591, 6593, 6596, 6601, 6603, 6604, 6605, 6614, 6627, 6629, 6631, 6634, 6648, 6653, 6655, 6660, 6680, 6681, 6682, 6684, 6687, 6688, 6689, 6693, 6694, 6695, 6697, 6705, 6706, 6707, 6709, 6712, 6714, 6715, 6717, 6720, 6728, 6732, 6734, 6736, 6739, 6743, 6744, 6746, 6754, 6762, 6763, 6764, 6765, 6768, 6769, 6774, 6775, 6776, 6780, 6797, 6806, 6810, 6822, 6829, 6838, 6841, 6848, 6851, 6862, 6863, 6865, 6867, 6874, 6880, 6882, 6884, 6890, 6909, 6911, 6921, 6928, 6929, 6930, 6931, 6937, 6948, 6966, 6997, 7002, 7004, 7005, 7014, 7017, 7018, 7020, 7023, 7028, 7030, 7034, 7047, 7055, 7056, 7059, 7072, 7078, 7079, 7083, 7092, 7095, 7103, 7110, 7112, 7114, 7118, 7124, 7130, 7136, 7147, 7148, 7150, 7153, 7167, 7168, 7173, 7180, 7183, 7184, 7185, 7188, 7193, 7194, 7205, 7209, 7211, 7219, 7223, 7232, 7240, 7248, 7249, 7250, 7252, 7267, 7269, 7276, 7277, 7280, 7281, 7283, 7292, 7294, 7298, 7300, 7308, 7310, 7317, 7320, 7325, 7327, 7336, 7343, 7345, 7349, 7354, 7370, 7382, 7383, 7410, 7413, 7414, 7418, 7430, 7433, 7438, 7441, 7447, 7453, 7454, 7455, 7456, 7459, 7460, 7461, 7466, 7472, 7473, 7475, 7476, 7479, 7481, 7484, 7486, 7488, 7490, 7499, 7501, 7503, 7506, 7510, 7517, 7519, 7520, 7527, 7529, 7538, 7541, 7552, 7558, 7562, 7574, 7575, 7583, 7587, 7594, 7604, 7620, 7624, 7631, 7635, 7637, 7639, 7640, 7651, 7661, 7668, 7672, 7673, 7674, 7677, 7680, 7681, 7682, 7695, 7700, 7704, 7711, 7727, 7736, 7750, 7755, 7758, 7764, 7765, 7768, 7773, 7779, 7782, 7791, 7792, 7798, 7800, 7810, 7815, 7821, 7828, 7839, 7841, 7844, 7852, 7853, 7854, 7857, 7864, 7866, 7870, 7875, 7881, 7890, 7895, 7898, 7901, 7912, 7916, 7920, 7928, 7930, 7938, 7940, 7954, 7957, 7966, 7977, 7979, 7986, 7987, 7988, 7990, 7993, 7997, 8004, 8023, 8025, 8043, 8044, 8055, 8056, 8057, 8061, 8062, 8067, 8072, 8073, 8076, 8078, 8083, 8085, 8086, 8088, 8094, 8106, 8108, 8111, 8114, 8117, 8124, 8139, 8140, 8144, 8150, 8151, 8158, 8163, 8164, 8169, 8171, 8177, 8182, 8183, 8196, 8199, 8200, 8201, 8203, 8210, 8217, 8222, 8223, 8231, 8234, 8240, 8241, 8253, 8254, 8261, 8263, 8267, 8272, 8274, 8275, 8277, 8279, 8283, 8284, 8286, 8289, 8292, 8293, 8295, 8302, 8305, 8311, 8314, 8317, 8319, 8323, 8328, 8335, 8339, 8342, 8343, 8346, 8348, 8351, 8360, 8361, 8366, 8367, 8369, 8370, 8372, 8380, 8383, 8388, 8393, 8395, 8397, 8403, 8413, 8421, 8423, 8424, 8428, 8433, 8438, 8446, 8449, 8457, 8462, 8464, 8467, 8475, 8477, 8484, 8485, 8487, 8491, 8493, 8494, 8495, 8498, 8501, 8507, 8509, 8529, 8530, 8533, 8541, 8543, 8548, 8550, 8556, 8557, 8558, 8559, 8560, 8564, 8565, 8567, 8568, 8569, 8574, 8576, 8578, 8582, 8589, 8598, 8608, 8609, 8613, 8618, 8621, 8628, 8632, 8636, 8639, 8640, 8642, 8643, 8653, 8654, 8656, 8659, 8661, 8673, 8677, 8679, 8681, 8686, 8688, 8690, 8692, 8701, 8704, 8705, 8706, 8719, 8721, 8729, 8741, 8746, 8749, 8755, 8762, 8770, 8776, 8777, 8778, 8787, 8790, 8799, 8802, 8807, 8809, 8811, 8821, 8824, 8829, 8833, 8834, 8837, 8853, 8860, 8864, 8871, 8873, 8875, 8876, 8879, 8880, 8885, 8889, 8892, 8894, 8897, 8898, 8905, 8914, 8921, 8922, 8926, 8927, 8928, 8929, 8930, 8941, 8944, 8948, 8952, 8958, 8962, 8966, 8967, 8979, 8990, 8997, 9000, 9003, 9004, 9005, 9012, 9016, 9017, 9027, 9029, 9047, 9051, 9057, 9059, 9064, 9073, 9080, 9092, 9096, 9097, 9106, 9112, 9113, 9115, 9124, 9134, 9138, 9140, 9145, 9147, 9148, 9149, 9151, 9153, 9159, 9161, 9163, 9176, 9196, 9197, 9207, 9212, 9219, 9220, 9224, 9227, 9228, 9233, 9239, 9245, 9249, 9251, 9262, 9271, 9276, 9284, 9287, 9291, 9293, 9306, 9311, 9312, 9315, 9320, 9331, 9333, 9334, 9341, 9344, 9345, 9350, 9358, 9376, 9382, 9383, 9388, 9389, 9395, 9396, 9421, 9422, 9439, 9442, 9451, 9456, 9483, 9495, 9496, 9509, 9519, 9522, 9526, 9539, 9547, 9549, 9551, 9565, 9570, 9579, 9582, 9586, 9596, 9601, 9602, 9604, 9605, 9609, 9611, 9616, 9620, 9621, 9627, 9628, 9629, 9637, 9649, 9650, 9654, 9658, 9661, 9662, 9668, 9671, 9675, 9677, 9689, 9698, 9700, 9712, 9715, 9720, 9724, 9739, 9741, 9742, 9753, 9756, 9759, 9765, 9771, 9777, 9778, 9779, 9783, 9786, 9787, 9794, 9796, 9797, 9803, 9804, 9811, 9812, 9823, 9826, 9828, 9831, 9838, 9841, 9849, 9851, 9854, 9855, 9860, 9875, 9879, 9887, 9893, 9898, 9901, 9904, 9905, 9913, 9928, 9933, 9939, 9942, 9944, 9951, 9952, 9953, 9954, 9957, 9963, 9972, 9979, 9982, 9985, 9997, 10001, 10008, 10022, 10028, 10032, 10035, 10040, 10049, 10054, 10057, 10061, 10067, 10071, 10080, 10082, 10088, 10093, 10096, 10097, 10102, 10107, 10118, 10120, 10126, 10136, 10137, 10144, 10148, 10153, 10154, 10159, 10169, 10173, 10174, 10178, 10183, 10188, 10190, 10197, 10198, 10207, 10220, 10223, 10230, 10236, 10237, 10250, 10259, 10271, 10277, 10282, 10289, 10293, 10297, 10306, 10318, 10320, 10329, 10331, 10336, 10341, 10345, 10357, 10367, 10369, 10371, 10377, 10389, 10390, 10403, 10406, 10409, 10417, 10420, 10425, 10438, 10451, 10460, 10466, 10467, 10480, 10482, 10485, 10492, 10497, 10499, 10501, 10502, 10508, 10509, 10546, 10552, 10558, 10563, 10581, 10596, 10600, 10608, 10619, 10622, 10626, 10628, 10629, 10641, 10644, 10645, 10647, 10664, 10667, 10668, 10674, 10679, 10682, 10683, 10691, 10703, 10708, 10715, 10720, 10722, 10723, 10733, 10747, 10748, 10753, 10756, 10783, 10793, 10795, 10797, 10803, 10815, 10826, 10839, 10841, 10842, 10846, 10853, 10859, 10868, 10908, 10915, 10922, 10926, 10938, 10939, 10941, 10942, 10944, 10947, 10950, 10956, 10964, 10967, 10975, 10979, 10982, 10991, 10997, 11002, 11007, 11009, 11018, 11027, 11032, 11038, 11040, 11047, 11060, 11073, 11080, 11082, 11083, 11095, 11105, 11106, 11110, 11113, 11116, 11119, 11129, 11131, 11134, 11143, 11167, 11169, 11172, 11174, 11178, 11181, 11186, 11202, 11208, 11219, 11221, 11223, 11224, 11226, 11228, 11230, 11233, 11235, 11246, 11257, 11268, 11272, 11281, 11287, 11295, 11298, 11303, 11310, 11311, 11314, 11327, 11330, 11341, 11345, 11350, 11354, 11358, 11377, 11381, 11388, 11395, 11400, 11401, 11406, 11408, 11413, 11415, 11416, 11419, 11420, 11426, 11431, 11437, 11444, 11453, 11458, 11466, 11482, 11483, 11490, 11501, 11504, 11506, 11507, 11522, 11526, 11537, 11539, 11545, 11549, 11552, 11561, 11569, 11574, 11581, 11583, 11584, 11589, 11591, 11594, 11596, 11599, 11624, 11633, 11637, 11643, 11654, 11659, 11662, 11663, 11672, 11673, 11677, 11680, 11685, 11687, 11691, 11699, 11703, 11709, 11711, 11720, 11728, 11729, 11733, 11744, 11752, 11755, 11762, 11771, 11776, 11783, 11797, 11801, 11804, 11811, 11814, 11821, 11823, 11824, 11831, 11844, 11847, 11850, 11854, 11860, 11866, 11867, 11870, 11873, 11886, 11889, 11906, 11908, 11912, 11916, 11918, 11925, 11930, 11931, 11932, 11938, 11944, 11946, 11961, 11965, 11968, 11973, 11977, 11984, 11986, 11989, 11995, 12008, 12014, 12017, 12019, 12021, 12023, 12037, 12041, 12043, 12046, 12049, 12053, 12062, 12069, 12071, 12074, 12079, 12093, 12096, 12102, 12104, 12109, 12114, 12115, 12119, 12120, 12127, 12129, 12130, 12136, 12140, 12145, 12146, 12147, 12153, 12154, 12157, 12160, 12165, 12166, 12168, 12174, 12176, 12179, 12185, 12189, 12195, 12198, 12218, 12223, 12224, 12225, 12227, 12232, 12235, 12236, 12253, 12256, 12260, 12271, 12273, 12279, 12285, 12288, 12290, 12294, 12306, 12307, 12308, 12310, 12320, 12323, 12324, 12325, 12326, 12329, 12334, 12335, 12343, 12349, 12350, 12352, 12353, 12358, 12362, 12373, 12374, 12382, 12390, 12397, 12400, 12408, 12413, 12419, 12421, 12434, 12435, 12438, 12444, 12452, 12454, 12462, 12464, 12475, 12480, 12491, 12504, 12514, 12523, 12525, 12526, 12535, 12536, 12542, 12543, 12545, 12552, 12555, 12557, 12560, 12573, 12575, 12617, 12628, 12639, 12722, 12768, 12804 };
		m_gripVerts = backVerts;
		m_gripPos.setZero(m_gripVerts.size(), 3);
		for (int i = 0; i < m_gripVerts.size(); i++) {
			m_gripPos.row(i) = verts.row(m_gripVerts[i]);
			m_gripPos(i, 1) += 3.5;
		}
	}

	/*
	Before the mesh is rendered, do a time step of the simulator and hand over the updated
	vertex positions to the viewer.
	Note that you would want to directly map the vertex positions into the OpenGL buffer
	in a high-performance setting, instead of passing them like this.
	*/
	bool pre_draw() {
		if (m_sim) {
			m_simTimer.startStopWatch();
			m_sim->step(m_numIterations);
			m_simTimer.stopStopWatch();
			viewer->data().set_mesh(m_sim->getPositions().block(0,0,m_numVertices,3), m_faces);
			return false;
		}
		return false;
	}

	/*
		Customized key events recognized by the libIGL viewer
	*/
	bool key_pressed(unsigned int key, int modifiers)
	{
		if (key == 49) {
			// Print FPS
			int avgMs = m_simTimer.evaluateAverage() / 1000;
			std::cout << "Average ms for time-step (includes vertex position updates, but not rendering): " << avgMs << std::endl;
			std::cout << "That is, the simulation runs at " << (1000 / avgMs) << " FPS" << std::endl;
		}
		else if (key == 50) {
			// reset positions
			if (m_sim) {
				m_sim->resetPositions();
			}
			m_isGripping = false;
		}
		else if (key == 51) {
			// hang mesh high up at back vertices or release it (toggle)
			if (m_sim) {
				if (!m_isGripping) {
					m_sim->setGrip(m_gripVerts, m_gripPos);
					m_isGripping = true;
				}
				else {
					m_sim->releaseGrip();
					m_isGripping = false;
				}
			}
		}
		else if (key == 52) {
			// toggle stiffness between normal and super elastic
			if (m_sim) {
				if (!m_toggleStiff) {
					m_sim->setStiffnessFactor(0.2);
					m_toggleStiff = true;
				}
				else {
					m_sim->setStiffnessFactor(1);
					m_toggleStiff = false;
				}
			}
		}
		else if (key == 53) {
			// toggle between bullet time and real time
			if (m_sim) {
				if (!m_toggleSlowmo) {
					m_sim->changeTimeStep(6);
					m_toggleSlowmo = true;
				}
				else {
					m_sim->changeTimeStep(16);
					m_toggleSlowmo = false;
				}
			}
		}
		else if (key == 54) {
			// Reset simulation (i.e. complete recomputation)
			if (m_sim) {
				delete m_sim;
				m_sim = initSimulator(m_verts, m_faces, m_velos, m_url);
			}
			m_isGripping = false;
			m_toggleStiff = false;
			m_toggleSlowmo = false;
		}
		return true;
	}


};

/*
Here we load a mesh, initialize the simulation and a viewer and start the main
render/timestep loop.
*/
int main()
{
	// Depending on whatever your default working directory is and wherever this mesh
	// file is, you will need to change this URL
	std::string meshURL = "armadillo.obj";

	// Load a mesh using IGL
	PD::PDPositions verts, velos;
	PD::PDTriangles faces;
	if (igl::readOBJ(meshURL, verts, faces)) {
		int numVertices = verts.rows();
		velos.setZero(numVertices, 3);

		// Number of local/global iterations in the reduced projective dynamics solver
		int numIterations = 10;

		// Start a viewer that uses a simple plugin to draw the simulated mesh
		// (it does numIterations local/global steps to simulate the next timestep
		// before drawing the mesh).
		SimViewer simViewer(verts, faces, velos, meshURL, numIterations);
		igl::opengl::glfw::Viewer viewer;
		viewer.data().set_mesh(verts, faces);
		viewer.core().is_animating = true;
		viewer.plugins.push_back(&simViewer);
		viewer.init_plugins();
		viewer.launch();
	}
	else {
		std::cout << "Could not load mesh! Put the mesh file into the working directory (VS project directory or next to the HRPD.exe)." << std::endl;
		system("pause");
		return -1;
	}

    return 0;
}

