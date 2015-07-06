#include "pch_bcl.h"
#include "PlayerProxy.h"
#include <PrimitiveVisualizer.h>
#include <fstream>
#include <Eigen\fft>
#include "CCA.h"
#include "EigenExtension.h"
#include "ArmatureBlock.h"
#include <algorithm>
#include <boost\multi_array.hpp>
#include "AnimationAnalyzer.h"
#include <ppl.h>

using namespace Causality;
using namespace Eigen;
using namespace std;

BlockArmature		g_PlayeerBlocks;
static const float	PcaCutoff = 0.04f; // 0.2^2
static const float	EnergyCutoff = 0.3f;

static const float	CharacterPcaCutoff = 0.01f; // 0.2^2
static const float	CharacterEnergyCutoff = 0.3f;
static const float	alpha = 0.8f;

bool				g_EnableInputFeatureLocalization = true;
bool				g_EnableDebugLogging = false;
static const char*  DefaultAnimationSet = "walk";


void PlayerProxy::StreamPlayerFrame(const TrackedBody& body, const TrackedBody::FrameType& frame)
{
	using namespace Eigen;
	using namespace DirectX;
	std::lock_guard<std::mutex> guard(BufferMutex);

	FeatureBuffer.push_back();

	if (FeatureBuffer.size() > RecordFeatures)
		FeatureBuffer.pop_front(); // everything works fine beside it invaliad the memery...

	if (!FeatureBuffer.is_linearized())
		FeatureBuffer.linearize();

	auto& fb = FeatureBuffer.back();
	float* vs = reinterpret_cast<float*>(fb.data());

	for (const auto& block : g_PlayeerBlocks)
	{
		auto vsi = RowVectorXf::Map(
			vs + block->AccumulatedJointCount * InputFeature::Dimension,
			block->GetFeatureDim<InputFeature>());

		vsi = block->GetFeatureVector<InputFeature>(frame, g_EnableInputFeatureLocalization);
	}
}

void PlayerProxy::ResetPlayer(TrackedBody * pOld, TrackedBody * pNew)
{
	ClearPlayerFeatureBuffer();
	SetActiveController(-1);
}

Eigen::Map<PlayerProxy::FeatureMatrixType> PlayerProxy::GetPlayerFeatureMatrix(time_seconds duration)
{
	using namespace std;
	int si = duration.count() * 30;
	if (FeatureBuffer.size() >= si)
	{
		std::lock_guard<mutex> guard(BufferMutex);
		si = min(max(si, 0), (int)FeatureBuffer.size());
		auto sidx = FeatureBuffer.size() - si;
		auto head = &FeatureBuffer[sidx][0];
		return FeatureMatrixType::Map(head, si, FeatureMatrixType::ColsAtCompileTime);
	}
	else
	{
		return FeatureMatrixType::Map((float*)nullptr, 0, FeatureMatrixType::ColsAtCompileTime);
	}
}

class CcaArmatureTransform : public ArmatureTransform
{
public:
	struct CcaMap
	{
		DenseIndex Jx, Jy;
		MatrixXf A, B;
		RowVectorXf uX, uY;
		JacobiSVD<MatrixXf> svdBt;
		MatrixXf invB;
		bool useInvB;
	};

	struct PcaCcaMap : public CcaMap
	{
		MatrixXf	pcX, pcY;
		RowVectorXf uXpca, uYpca;
	};

	vector<PcaCcaMap> Maps;

public:
	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override
	{
		//const auto dF = frame_type::StdFeatureDimension;
		//const auto dY = 3;
		//RowVectorXf X(source_frame.size() * dF);
		//RowVectorXf Y(target_frame.size() * dF);
		//source_frame.PopulateStdFeatureVector(X);
		//target_frame.PopulateStdFeatureVector(Y); //! populate non-mapped with default values

		//for (const auto& map : Maps)
		//{
		//	auto& Xp = X.middleCols<dF>(map.Jx * dF);
		//	auto& Yp = Y.middleCols<dY>(map.Jy * dF);
		//	ApplyCcaMap(map, Xp, Yp);
		//}

		//target_frame.RebuildFromStdFeatureVector(Y, *pTarget);
	}

	template <class DerivedX, class DerivedY>
	static void ApplyCcaMap(_In_ const CcaMap& map, _In_ const DenseBase<DerivedX> &Xp, _Out_ DenseBase<DerivedY> &Yp)
	{
		auto U = ((Xp.rowwise() - map.uX) * map.A).eval();
		if (map.useInvB)
			Yp = U * map.invB;
		else
			Yp = map.svdBt.solve(U.transpose()).transpose(); // Y' ~ B' \ U'
		Yp.rowwise() += map.uY;
	}

	template <class DerivedX, class DerivedY>
	static void ApplyPcaCcaMap(_In_ const PcaCcaMap& map, _In_ const DenseBase<DerivedX> &Xp, _Out_ MatrixBase<DerivedY> &Yp)
	{
		// Project X with PCA
		auto Xpca = ((Xp.rowwise() - map.uXpca) * map.pcX).eval();
		// Project Xpca with CCA to latent space
		auto U = ((Xpca.rowwise() - map.uX) * map.A).eval();
		// Recover Y from latent space
		if (map.useInvB)
			Yp = U * map.invB;
		else
			Yp = map.svdBt.solve(U.transpose()).transpose(); // Y' ~ B' \ U'
		// Add the mean
		Yp.rowwise() += map.uY;
		// Reconstruct by principle components
		Yp *= map.pcY.transpose();
		Yp.rowwise() += map.uYpca;
	}
};

class BlockizedCcaArmatureTransform : public CcaArmatureTransform
{
public:
	const BlockArmature *pSblocks, *pTblocks;
	virtual void Transform(_Out_ frame_type& target_frame, _In_ const frame_type& source_frame) const override
	{
		const auto& sblocks = *pSblocks;
		const auto& tblocks = *pTblocks;
		for (const auto& map : Maps)
		{
			auto pSb = sblocks[map.Jx];
			auto pTb = tblocks[map.Jy];
			auto X = pSb->GetFeatureVector<InputFeature>(source_frame, g_EnableInputFeatureLocalization);
			auto Y = pTb->GetFeatureVector<CharacterFeature>(target_frame);

			ApplyPcaCcaMap(map, X, Y);

			//cout << " X = " << X << endl;
			//cout << " Yr = " << Y << endl;
			pTb->SetFeatureVector<CharacterFeature>(target_frame, Y);
		}

		//target_frame[0].LclTranslation = source_frame[0].GblTranslation;
		//target_frame[0].LclRotation = source_frame[0].LclRotation;
		target_frame.RebuildGlobal(*pTarget);
	}
};

pair<JointType, JointType> XFeaturePairs[] = {
	{JointType_SpineBase, JointType_SpineShoulder},
	{ JointType_SpineShoulder, JointType_Head },
	{JointType_ShoulderLeft, JointType_ElbowLeft},
	{JointType_ShoulderRight, JointType_ElbowRight },
	{ JointType_ElbowLeft, JointType_HandLeft },
	{ JointType_ElbowRight, JointType_HandRight },
	{ JointType_HipLeft, JointType_KneeLeft},
	{ JointType_HipRight, JointType_KneeRight},
	{ JointType_KneeLeft, JointType_AnkleLeft},
	{ JointType_KneeRight, JointType_AnkleRight},
	//{ JointType_HandLeft, JointType_HandTipLeft },
	//{ JointType_HandRight, JointType_HandTipRight },
	//{ JointType_HandLeft, JointType_ThumbLeft },
	//{ JointType_HandRight, JointType_ThumbRight },
};

JointType KeyJoints[] = {
	JointType_SpineBase,		//1
	JointType_SpineShoulder,	//2
	JointType_Head,				//3
	JointType_ShoulderLeft,		//4
	JointType_ElbowLeft,		//5
	JointType_WristLeft,		//6
	JointType_ShoulderRight,	//7
	JointType_ElbowRight,		//8
	JointType_WristRight,		//9
	JointType_HipLeft,			//10
	JointType_KneeLeft,			//11
	JointType_AnkleLeft,		//12
	JointType_HipRight,			//13
	JointType_KneeRight,		//14
	JointType_AnkleRight,		//15
};

float BoneRadius[JointType_Count] = {

};

//static const size_t KeyJointCount = ARRAYSIZE(KeyJoints);
//
//static const size_t FeatureCount = (KeyJointCount * (KeyJointCount - 1)) / 2;
//static const size_t FeatureDim = FeatureCount * 3;

VectorXf HumanFeatureFromFrame(const AffineFrame& frame)
{
	VectorXf X(ARRAYSIZE(XFeaturePairs) * 3);
	for (size_t i = 0; i < ARRAYSIZE(XFeaturePairs); i++)
	{
		auto& p = XFeaturePairs[i];
		Vector3 v = frame[p.first].GblTranslation - frame[p.second].GblTranslation;
		X.block<3, 1>(i * 3, 0) = Map<Vector3f>(&v.x);
	}

	//VectorXf X(FeatureDim);
	//int k = 0;
	//Vector3 v;
	//for (size_t i = 0; i < KeyJointCount; i++)
	//{
	//	for (size_t j = i+1; j < KeyJointCount; j++)
	//		if (i != j)
	//		{
	//			v = frame[KeyJoints[i]].GblTranslation - frame[KeyJoints[j]].GblTranslation;
	//			X.block<3, 1>(k++ * 3,0) = Map<Vector3f>(&v.x);
	//		}
	//}
	return X;
}

PlayerProxy::PlayerProxy()
	: IsInitialized(false),
	playerSelector(nullptr),
	CurrentIdx(-1),
	FeatureBuffer(FeatureBufferCaptcity)
{
	pKinect = Devices::KinectSensor::GetForCurrentView();
	pPlayerArmature = &pKinect->Armature();
	if (g_PlayeerBlocks.empty())
	{
		g_PlayeerBlocks.SetArmature(*pPlayerArmature);
	}

	auto fReset = std::bind(&PlayerProxy::ResetPlayer, this, placeholders::_1, placeholders::_2);
	playerSelector.SetPlayerChangeCallback(fReset);

	auto fFrame = std::bind(&PlayerProxy::StreamPlayerFrame, this, placeholders::_1, placeholders::_2);
	playerSelector.SetFrameCallback(fFrame);

	playerSelector.Initialize(pKinect.get(), TrackedBodySelector::SelectionMode::Closest);

	Register();
	IsInitialized = true;
}

PlayerProxy::~PlayerProxy()
{
	Unregister();
	//std::ofstream fout("handpos.txt", std::ofstream::out);

	//fout.close();
}

void PlayerProxy::AddChild(SceneObject* pChild)
{
	SceneObject::AddChild(pChild);
	auto pChara = dynamic_cast<CharacterObject*>(pChild);
	if (pChara)
	{
		Controllers.emplace_back();
		auto& controller = Controllers.back();
		controller.ID = Controllers.size() - 1;
		controller.Initialize(*pPlayerArmature, *pChara);
		pChara->SetOpticity(1.0f);
	}
}

const static Eigen::IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

void PlayerProxy::SetActiveController(int idx)
{
	if (idx >= 0)
		idx = idx % Controllers.size();

	for (auto& c : Controllers)
	{
		if (c.ID != idx)
		{
			auto& chara = c.Character();
			chara.StartAction(DefaultAnimationSet);
			chara.SetOpticity(0.5f);
			if (c.ID == CurrentIdx && CurrentIdx != idx)
				chara.SetPosition(c.CMapRefPos);
		}
	}

	if (CurrentIdx != idx)
	{
		CurrentIdx = idx;
		if (CurrentIdx != -1)
		{
			auto& controller = GetController(CurrentIdx);
			auto& chara = controller.Character();
			chara.StopAction();
			chara.SetOpticity(1.0f);
			controller.MapRefPos = playerSelector->CurrentFrame()[0].GblTranslation;
			controller.CMapRefPos = chara.GetPosition();

		}
	}
}

int PlayerProxy::MapCharacterByLatestMotion()
{
	static const size_t FilterStregth = 6U; //Applies 4 iterates of Laplicaian filter
	static const size_t CropMargin = 1 * SAMPLE_RATE; // Ignore most recent 1 sec
	static const float Cutoff_Correlation = 0.5f;
	time_seconds InputDuration(10);

	auto& player = *playerSelector;
	static ofstream flog;
	if (g_EnableDebugLogging)
		flog.open("Xlog.txt");

	MatrixXf X = GetPlayerFeatureMatrix(InputDuration);
	//? WARNNING! GetPlayerFeatureMatrix() is ROW Major!!!
	//? But we have convert it into Column Major! Yes and No!!!
	auto N = X.rows();
	auto K = X.cols();

	if (N*K == 0) return -1; // Feature Matrix is not ready yet

	cout << "X : min = " << X.minCoeff() << " , max = " << X.maxCoeff() << endl;



	MatrixXcf Xf(N, K);

	concurrency::parallel_for(0, (int)K, [&Xf, &X](auto i)
		//for (size_t i = 0; i < K; i++)
	{
		//! Smooth the input 
		laplacian_smooth(Xf.col(i), FilterStregth);

		FFT<float> fft;
		fft.fwd(Xf.col(i).data(), X.col(i).data(), Xf.rows());
	}
	);

	cout << "X : min = " << X.minCoeff() << " , max = " << X.maxCoeff() << endl;
	//flog << X << endl;
	//flog.close();

	MatrixXf Xs = Xf.cwiseAbs2();

	//? How to normalize enery term between position and angular? express arg as arc length???
	RowVectorXf Eu3 = Xs.middleRows(MinHz, MaxHz).colwise().sum();
	RowVectorXf Euj = MatrixXf::Map(Eu3.data(), JointDemension, K / JointDemension).colwise().sum();

	int idx;
	VectorXf Ea = Xs.middleRows(MinHz - 1, MaxHz + 2).rowwise().sum();
	cout << Ea.transpose() << endl;

	Ea.segment(1, Ea.size() - 2).maxCoeff(&idx); // Frequency 3 - 30
	++idx;

	auto Ex = Ea.middleRows<3>(idx - 1);
	idx += MinHz;
	cout << Ex.transpose() << endl;

	Vector3f Ix = { idx - 1.0f, (float)idx, idx + 1.0f };
	float peekFreq = Ex.dot(Ix) / Ex.sum();

	size_t T = ceil(N / peekFreq);
	//! Retive Primary Time Period and Peek Frequency

	//! Crop and Resample the sequence
	int nT = max(5, (int)(N / T));
	Xs = cublic_bezier_resample(
		X.middleRows(N - (2 * T + CropMargin + 1), T * 2),
		StretchedSampleCount * 2,
		Eigen::CloseLoop);

	if (flog.is_open())
	{
		flog << Xs << endl;
		flog.close();
	}

	T = StretchedSampleCount; // Since we have resampled , Time period "T" is now equals to StretchedSampleCount
	int Ti = 1; // 2
	int Ts = T / Ti;

	int Ju = g_PlayeerBlocks.size();

	// Spatial Traits
	Eigen::Matrix<float, -1, -1> Xsp(3, Ju);
	vector<vector<MeanThinQr<MatrixXf>>> QrXs(Ts);
	vector<vector<Pca<MatrixXf>>> PcaXs(Ts);
	for (auto& v : QrXs)
		v.resize(Ju);
	for (auto& v : PcaXs)
		v.resize(Ju);


	VectorXf Eub(Ju);
	// Fuck it... 11,250,000 times of CCA(SVD) computation every guess
	// 1,406,250 SVD for 5x5 setup, much more reasonable!
	//for (auto i : Juk) //? <= 25 joint per USER?
	concurrency::parallel_for(0, (int)Ju, [&](auto i)
	{
		const auto& block = *g_PlayeerBlocks[i];
		auto stCol = block.AccumulatedJointCount * JointDemension;
		auto cols = block.Joints.size() * JointDemension;

		//? How to normalize muilti-joint block's energy???? 
		//? Use max or sum ??? - max seems like a better idea!
		Eub(i) = Euj.middleCols(stCol / JointDemension, cols / JointDemension).maxCoeff();

		auto Xk = Xs.middleCols(stCol, cols);

		//? Spatial Traits Calculation!!!
		Xsp.col(i) = Xk.rightCols<3>().colwise().mean().transpose();

		for (DenseIndex phi = 0; phi < T; phi += Ti)	//? <= 50 , we should optimze phase shifting search from rough to fine
		{
			auto Xp = Xk.middleRows(T - phi, T);
			auto& pca = PcaXs[phi / Ti][i];
			pca.compute(Xp, true);
			auto d = pca.reducedRank(PcaCutoff);
			QrXs[phi / Ti][i].compute(pca.coordinates(d), true);
		}
	}
	);

	//for (auto& block : boost::make_iterator_range(g_PlayeerBlocks.rbegin(),g_PlayeerBlocks.rend()))
	//{
	//	if (!g_EnableInputFeatureLocalization && block->parent() != nullptr)
	//	{
	//		Xsp.col(block->Index) -= Xsp.col(block->parent()->Index);
	//	}
	//}

	Xsp.colwise().normalize();
	//// Spatial Traits
	//Eigen::Matrix<float, -1, -1> Xsp(3, Ju);
	//{
	//	AnimationAnalyzer anlyzer(g_PlayeerBlocks);
	//	anlyzer.ComputeSpatialTraits()
	//}

	Eub = Eub.cwiseSqrt();
	Eub /= Eub.maxCoeff();

	vector<int> Juk;

	for (size_t i = 0; i < Ju; i++)
	{
		if (Eub(i) > EnergyCutoff)
		{
			Xsp.col(Juk.size()) = Xsp.col(i);
			Eub(Juk.size()) = Eub(i);
			Juk.push_back(i);
		}
	}
	Xsp.conservativeResize(Xsp.rows(),Juk.size());
	Eub.conservativeResize(Juk.size());
	MatrixXf XDir(2 * T * 3, Juk.size());
	for (size_t i = 0; i < Juk.size(); i++)
	{
		static_assert(JointDemension == 3, "This code segments assums Joint Feature is X-Y-Z relative position");
		auto& block = *g_PlayeerBlocks[Juk[i]];
		auto stCol = block.AccumulatedJointCount * JointDemension;
		auto cols = block.Joints.size() * JointDemension;
		stCol = stCol + cols - JointDemension;

		auto vs = Matrix<float, 3, -1>::Map(XDir.col(i).data(), 3, 2 * T);
		vs = Xs.middleCols<JointDemension>(stCol).transpose();// 2*t x 3
		vs.colwise().normalize();
	}

	//? HACK Juk Jck! Pre-reduce DOF
	//size_t Juk[] = { JointType_Head,JointType_ElbowLeft,JointType_ElbowRight,JointType_KneeLeft,JointType_KneeRight };

	//size_t Jck[] = { 8,12,17,22,35,40,45,50,54};
	//for (auto& jc : Jck)
	//	jc -= 1;

	////? HACK Ju
	//Ju = std::size(Juk);



	for (auto& controller : Controllers)			//? <= 5 character
	{
		auto& object = controller.Character();
		auto& chraraBlocks = object.Behavier().Blocks();
		size_t Jc = chraraBlocks.size();

		controller.SpatialMotionScore = numeric_limits<float>::min();
		auto& anim = object.Behavier()[DefaultAnimationSet];
		//for (auto& anim : object.Behavier().Clips())	//? <= 5 animation per character
		{
			auto& analyzer = controller.GetAnimationInfo(anim.Name);

			// Caculate Ecb, Energy of Character Blocks
			auto Ecb = analyzer.Eb;

			vector<int> Jck;
			for (size_t i = 0; i < Jc; i++)
			{
				if (Ecb(i) > CharacterEnergyCutoff)
				{
					Ecb(Jck.size()) = Ecb(i);
					Jck.push_back(i);
				}
			}
			Ecb.conservativeResize(Jck.size());

			// Alternate Spatial traits
			Matrix<float, -1, -1> Csp(3, Jck.size());
			MatrixXf Csp2(3 * T, Jck.size());
			for (size_t i = 0; i < Jck.size(); i++)
			{
				Csp.col(i) = analyzer.Sp.block<3,1>(0,Jck[i]);
				Csp2.col(i) = analyzer.Dirs.col(Jck[i]);
			}

			// Score for spatial traits !!!
			MatrixXf Asp = Xsp.transpose() * Csp;

			// Memery allocation
			auto CoRSize = Juk.size() + Jck.size();
			std::vector<MatrixXf> Rm(Ts);
			for (auto& m : Rm)
			{
				m.setZero(CoRSize, CoRSize);
			}

			//MatrixXf CoR;//(Ju + Jc, Ju + Jc);
			//CoR.setZero(CoRSize, CoRSize);


			VectorXf mScores(Ts);
			vector<vector<DenseIndex>> mMatchings(Ts);
			//DenseIndex phi = 0;

			concurrency::parallel_for(0, (int)T, Ti, [&](auto phi)
				//for (DenseIndex phi = 0; phi < T; phi += Ti)	//? <= 50 , we should optimze phase shifting search from rough to fine
			{
				auto phidx = phi / Ti;

				auto Xp = Xs.middleRows(T - phi, T);


				auto A = Rm[phidx].topLeftCorner(Juk.size(), Jck.size());


				for (int i = 0; i < Juk.size(); ++i)
				{
					int uid = Juk[i];
					auto& qrX = QrXs[phi / Ti][uid];

					for (int j = 0; j < Jck.size(); j++)
					{
						int cid = Jck[j];
						auto& qrY = analyzer.Qrs[cid];
						float r = 0;
						if (qrX.rank() * qrY.rank() != 0)
						{
							Cca<float> cca;
							cca.computeFromQr(qrX, qrY, true);	//? it's <= 6x6 SVD inside
							r = cca.correlaltions().minCoeff();
						}
						A(i, j) = r;
					}
				}

				MatrixXf Asp2 = XDir.middleRows((T - phi)*3, T*3).transpose() * Csp2;
				Asp2 /= (float)T;
				// Cutoff the none-correlated match
				// a = (a > cutoff) ? a : -1.0f
				A = (A.array() > Cutoff_Correlation).select(A, -1.0f);
				//A.noalias() = A * alpha + Asp;
				A.noalias() = A * alpha + Asp2;

				A.array() *= Eub.array().replicate(1, A.cols());
				A.array() *= Ecb.array().replicate(A.rows(), 1);

				//CoR.topLeftCorner(Ju, Jc) = A;

				//auto matching = max_weight_bipartite_matching(A.transpose());
				vector<DenseIndex> matching(A.cols());
				for (int k = 0; k < A.cols(); k++)
				{
					A.col(k).maxCoeff(&matching[k]);
				}

				auto score = matching_cost(A.transpose(), matching);// / A.cols();
				//auto score = matching_cost(A, matching);
				mScores[phidx] = score;
				mMatchings[phidx] = matching;
			}
			);

			float maxScore = numeric_limits<float>::min();
			DenseIndex maxPhi = -1;
			vector<DenseIndex> maxMatching;
			for (DenseIndex phi = 0; phi < T; phi += Ti)
			{
				auto phidx = phi / Ti;
				auto score = mScores[phidx];
				if (score > maxScore)
				{
					maxPhi = phi;
					maxMatching = mMatchings[phidx];
					maxScore = score;
				}
			}

			//cout << "Scores over Phi : \n" << mScores << endl;

			analyzer.BestMatchingScore = maxScore;
			analyzer.BestMatching = maxMatching;
			analyzer.BestPhi = maxPhi;

			cout << "=============================================" << endl;
			cout << "Best assignment for " << controller.Character().Name << " : " << anim.Name << endl;
			cout << "Scores : " << maxScore << endl;
			//int sdxass[] = {3,14,19,24,29};
			//for (size_t i = 0; i < size(sdxass); i++)
			//	maxMatching[Juk[i]] = sdxass[i];

			cout << "*********************************************" << endl;
			cout << "Human Skeleton Blocks : " << endl;
			for (auto i : Juk)
			{
				const auto& blX = *g_PlayeerBlocks[i];
				cout << "Block " << i << " = {";
				for (auto pJoint : blX.Joints)
				{
					cout << pJoint->Name() << ", ";
				}
				cout << "\b\b}" << endl;
			}

			cout << "*********************************************" << endl;
			cout << "Character " << controller.Character().Name << "'s Skeleton Blocks : " << endl;

			for (auto& i : Jck)
			{
				const auto& blY = *controller.Character().Behavier().Blocks()[i];
				cout << "Block " << i << " = {";
				for (auto pJoint : blY.Joints)
				{
					cout << pJoint->Name() << ", ";
				}
				cout << "\b\b}" << endl;
			}
			cout << "*********************************************" << endl;

			// Setup Binding from matching
			if (controller.SpatialMotionScore < maxScore)
			{
				cout << "New best : " << anim.Name << endl;
				controller.SpatialMotionScore = maxScore;
				auto pBinding = new BlockizedCcaArmatureTransform();
				controller.SetBinding(pBinding);
				pBinding->SetSourceArmature(*pPlayerArmature);
				pBinding->SetTargetArmature(controller.Character().Armature());
				pBinding->pSblocks = &g_PlayeerBlocks;
				pBinding->pTblocks = &controller.Character().Behavier().Blocks();

				//for (int i = 0; i < Juk.size(); ++i)
				for (int j = 0; j < Jck.size(); ++j)
				{
					DenseIndex Jx = maxMatching[j], Jy = j;
					//DenseIndex Jx = i, Jy = maxMatching[i];
					if (Jy == -1 || Jy >= Jc || Jx == -1 || Jx >= Jc) continue;
					Jx = Juk[Jx]; Jy = Jck[Jy];

					const auto& blX = *g_PlayeerBlocks[Jx];
					const auto& blY = *controller.Character().Behavier().Blocks()[Jy];

					//cout << pPlayerArmature->at(Jx)->Name() << " ==> " << controller.Character().Armature()[Jy]->Name() << endl;

					auto& qrX = QrXs[maxPhi / Ti][Jx];
					auto& pcaX = PcaXs[maxPhi / Ti][Jx];
					auto& qrY = analyzer.Qrs[Jy];
					auto& pcaY = analyzer.Pcas[Jy];

					Cca<float> cca;
					cca.computeFromQr(qrX, qrY, true);

					if (cca.rank() <= 0)
						continue;

					cout << '{';
					for (auto pJoint : blX.Joints)
					{
						cout << pJoint->Name() << ", ";
					}
					cout << "\b\b} ==> {";
					for (auto pJoint : blY.Joints)
					{
						cout << pJoint->Name() << ", ";
					}
					cout << "\b\b}" << endl;

					// populate map
					pBinding->Maps.emplace_back();
					auto& map = pBinding->Maps.back();
					map.Jx = Jx; map.Jy = Jy;
					map.A = cca.matrixA();
					map.B = cca.matrixB();
					map.uX = qrX.mean();
					map.uY = qrY.mean();
					map.uXpca = pcaX.mean();
					map.uYpca = pcaY.mean();
					map.pcX = pcaX.components(qrX.cols());
					map.pcY = pcaY.components(qrY.cols());

					if (cca.rank() == qrY.cols()) // d == dY
					{
						map.useInvB = true;
						map.invB = map.B.inverse();
					}
					else
					{
						map.useInvB = false;
						map.svdBt = JacobiSVD<MatrixXf>(map.B.transpose(), ComputeThinU | ComputeThinV);;
					}
				}
			}
		}

	}

	CharacterController* pControl = nullptr;
	for (auto& con : Controllers)
	{
		if (!pControl || con.SpatialMotionScore > pControl->SpatialMotionScore)
			pControl = &con;
	}
	if (!pControl) return -1;
	SetActiveController(pControl->ID);

	return pControl->ID;
}

//void PlayerProxy::LocalizePlayerFeature(Eigen::MatrixXf &X)
//{
//	for (const auto &pBlock : g_PlayeerBlocks) //? <= 25 joint per USER?
//	{
//		auto& block = *pBlock;
//		if (block.Joints[0]->is_root()) continue;
//		auto refId = block.Joints[0]->parent()->ID();
//		auto refCols = X.middleCols<JointDemension>(refId*JointDemension);
//		for (auto& pJ : block.Joints)
//		{
//			auto jid = pJ->ID();
//			X.middleCols<JointDemension>(jid*JointDemension) -= refCols;
//		}
//	}
//}

void PlayerProxy::ClearPlayerFeatureBuffer()
{
	std::lock_guard<std::mutex> guard(BufferMutex);
	FeatureBuffer.clear();
}

// Character Map State

bool PlayerProxy::IsMapped() const { return CurrentIdx >= 0; }

const CharacterController & PlayerProxy::CurrentController() const {
	for (auto& c : Controllers)
	{
		if (c.ID == CurrentIdx)
			return c;
	}
}

CharacterController & PlayerProxy::CurrentController() {
	for (auto& c : Controllers)
	{
		if (c.ID == CurrentIdx)
			return c;
	}
}

const CharacterController & PlayerProxy::GetController(int state) const {
	for (auto& c : Controllers)
	{
		if (c.ID == state)
			return c;
	}
}

CharacterController & PlayerProxy::GetController(int state)
{
	for (auto& c : Controllers)
	{
		if (c.ID == state)
			return c;
	}
}

void PlayerProxy::OnKeyUp(const KeyboardEventArgs & e)
{
	if (e.Key == VK_OEM_PERIOD || e.Key == '.' || e.Key == '>')
	{
		SetActiveController(CurrentIdx + 1);
	}
	else if (e.Key == VK_OEM_COMMA || e.Key == ',' || e.Key == '<')
	{
		SetActiveController(CurrentIdx - 1);
	}
}

void PlayerProxy::OnKeyDown(const KeyboardEventArgs & e)
{
}

//x DON"T USE THIS
pair<float, float> PlayerProxy::ExtractUserMotionPeriod()
{
	MatrixXf
		X(BufferFramesCount, JointCount * JointDemension);
	MatrixXcf
		Xf(BufferFramesCount, JointCount * JointDemension);
	MatrixXf
		JointReductedSpecturm(BufferFramesCount, JointCount);
	Array<float, JointCount, 1>		JointPeriod; // Period time in seconds, from fft Peek frequency

	FFT<float> fft;
	for (size_t i = 0; i < X.rows(); i++)
	{
		fft.fwd(Xf.row(i), X.row(i));
	}
	Xf = Xf.array() * Xf.array().conjugate();
	X = Xf.real().transpose();

	for (size_t i = 0; i < JointCount; i++)
	{
		Map<Matrix<float, JointCount, JointDemension>> Jsp(&X(0, i * JointDemension));
		// sum up spectrum energy for X-Y-Z components
		JointReductedSpecturm.col(i) = Jsp.rowwise().sum();
	}

	for (size_t j = 0; j < JointReductedSpecturm.rows(); j++)
	{
		int idx;
		// filter frequency with 3-15 reoccurence times
		JointReductedSpecturm.block<1, HzWidth>(j, MinHz).maxCoeff(&idx);
		JointPeriod[j] = 15.0f / (float)(idx + 3); // 450 frame/(30 frame/s)
	}

	auto overallSpectrum = JointReductedSpecturm.colwise().sum();

	int idx;
	overallSpectrum.block<1, HzWidth>(1, MinHz).maxCoeff(&idx);
	return make_pair(.0f, idx);
}

RenderFlags Causality::PlayerProxy::GetRenderFlags() const
{
	return RenderFlags::SpecialEffects;
}

//void PlayerProxy::PrintFrameBuffer(int No)
//{
//	int flag = ofstream::out | (No == 1 ? 0 : ofstream::app);
//	ofstream fout("handpos.csv", flag);
//
//	//Matrix<float, Dynamic, FeatureDim> X(BufferFramesCount);
//	for (auto& frame : FrameBuffer)
//	{
//		auto Xj = HumanFeatureFromFrame(frame);
//
//		for (size_t i = 0; i < Xj.size() - 1; i++)
//		{
//			fout << Xj(i) << ',';
//		}
//		fout << Xj(Xj.size() - 1) << endl;
//
//		//for (auto& bone : frame)
//		//{
//		//	const auto& pos = bone.GblTranslation;
//		//	//DirectX::XMVECTOR quat = bone.LclRotation;
//		//	//Vector3 pos = DirectX::XMQuaternionLn(quat);
//		//	fout << pos.x << ',' << pos.y << ',' << pos.z << ',';
//		//}
//		//fout << endl;
//	}
//	fout.close();
//}

void PlayerProxy::Update(time_seconds const & time_delta)
{
	using namespace std;
	using namespace Eigen;

	if (!IsInitialized || !playerSelector)
		return;

	static long long frame_count = 0;

	auto& player = *playerSelector;
	if (!player.IsTracked()) return;

	const auto& frame = player.PullLatestFrame();

	if (IsMapped())
	{
		auto& controller = CurrentController();
		controller.UpdateTargetCharacter(frame);
		return;
	}

	if (frame_count++ % SampleRate != 0)
	{
		return;
	}

	if (FeatureBuffer.size() >= 0)
	{
		auto idx = MapCharacterByLatestMotion();
		if (IsMapped())
		{
			CurrentController().Character().StopAction();
			cout << "Mapped!!!!!!!!!" << endl;
		}
	}
}

bool PlayerProxy::UpdateByFrame(const AffineFrame & frame)
{
	AffineFrame tframe;
	// Caculate likilihood
	for (int i = 0, end = Controllers.size(); i < end; ++i)
	{
		auto& obj = GetController(i);
		auto& binding = obj.Binding();
		auto& kobj = obj.Character();
		auto& armature = kobj.Armature();
		auto& space = kobj.Behavier();

		binding.Transform(obj.PotientialFrame, frame);
		Likilihood(i) = space.FrameLikilihood(frame);
	}

	// State evolution
	StateProbality.noalias() = TransferMatrix * StateProbality;

	// 
	StateProbality = StateProbality.cwiseProduct(Likilihood);
	float c = StateProbality.sum();
	StateProbality = StateProbality / c;

	for (size_t i = 0; i < Controllers.size(); i++)
	{
		GetController(i).Character().SetOpticity(StateProbality(i));
	}

	int maxIdx = -1;
	auto currentP = StateProbality.maxCoeff(&maxIdx);
	if (maxIdx != CurrentIdx)
	{
		int oldIdx = CurrentIdx;
		CurrentIdx = maxIdx;
		//StateChangedEventArgs arg = { oldIdx,maxIdx,1.0f,GetController(oldIdx),GetController(maxIdx) };
		//if (!StateChanged.empty())
		//	StateChanged(arg);
	}

	if (IsMapped())
	{
		auto& state = CurrentController();
		auto& cframe = state.Character().MapCurrentFrameForUpdate();
		state.Binding().Transform(cframe, frame);
		state.Character().ReleaseCurrentFrameFrorUpdate();
	}
	return true;
}

bool PlayerProxy::IsVisible(const DirectX::BoundingGeometry & viewFrustum) const
{
	return true;
}

void PlayerProxy::Render(RenderContext & context, DirectX::IEffect* pEffect)
{
	if (!playerSelector) return;
	auto& player = *playerSelector;

	Color color = DirectX::Colors::LimeGreen.v;

	if (IsMapped())
		color.A(0.3f);

	if (player.IsTracked())
	{
		const auto& frame = player.PullLatestFrame();
		if (&frame == nullptr) return;

		DrawArmature(*player.BodyArmature, frame, color);
	}
}

void XM_CALLCONV PlayerProxy::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	DirectX::Visualizers::g_PrimitiveDrawer.SetView(view);
	DirectX::Visualizers::g_PrimitiveDrawer.SetProjection(projection);
}

KinectVisualizer::KinectVisualizer()
{
	pKinect = Devices::KinectSensor::GetForCurrentView();
}

bool KinectVisualizer::IsVisible(const DirectX::BoundingGeometry & viewFrustum) const
{
	return true;
}

void KinectVisualizer::Render(RenderContext & context, DirectX::IEffect* pEffect)
{
	auto &players = pKinect->GetTrackedBodies();
	using DirectX::Visualizers::g_PrimitiveDrawer;

	for (auto& player : players)
	{
		if (player.IsTracked())
		{
			const auto& frame = player.PullLatestFrame();
			if (&frame == nullptr) return;

			DrawArmature(*player.BodyArmature, frame, DirectX::Colors::LimeGreen.v);
		}
	}
}

void XM_CALLCONV KinectVisualizer::UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	DirectX::Visualizers::g_PrimitiveDrawer.SetView(view);
	DirectX::Visualizers::g_PrimitiveDrawer.SetProjection(projection);
}

RenderFlags Causality::KinectVisualizer::GetRenderFlags() const
{
	return RenderFlags::SpecialEffects;
}

CharacterController::~CharacterController()
{
	for (auto& p : m_Analyzers)
	{
		delete p.second;
	}
}

void CharacterController::Initialize(const IArmature & player, CharacterObject & character)
{
	SetSourceArmature(player);
	SetTargetCharacter(character);
}

const ArmatureTransform & CharacterController::Binding() const { return *m_pBinding; }

ArmatureTransform & CharacterController::Binding() { return *m_pBinding; }

void CharacterController::SetBinding(ArmatureTransform * pBinding)
{
	m_pBinding.reset(pBinding);
}

const CharacterObject & CharacterController::Character() const { return *m_pCharacter; }

CharacterObject & CharacterController::Character() { return *m_pCharacter; }

void CharacterController::UpdateTargetCharacter(const AffineFrame & frame) const
{
	auto& cframe = m_pCharacter->MapCurrentFrameForUpdate();
	m_pBinding->Transform(cframe, frame);

	float l = 100;
	for (auto& bone :frame)
	{
		if (bone.GblTranslation.y < l)
			l = bone.GblTranslation.y;
	}

	auto pos = frame[0].GblTranslation - MapRefPos + CMapRefPos;

	m_pCharacter->SetPosition(pos);
	m_pCharacter->ReleaseCurrentFrameFrorUpdate();
}

AnimationAnalyzer & CharacterController::GetAnimationInfo(const string & name) {
	return *m_Analyzers[name];
}

void CharacterController::SetSourceArmature(const IArmature & armature) {
	if (m_pBinding)
		m_pBinding->SetSourceArmature(armature);
}

void CharacterController::SetTargetCharacter(CharacterObject & object) {
	m_pCharacter = &object;
	if (m_pBinding)
		m_pBinding->SetTargetArmature(object.Armature());
	auto & behavier = object.Behavier();
	PotientialFrame = object.Armature().default_frame();

	using namespace concurrency;
	vector<task<void>> tasks;
	for (auto& anim : object.Behavier().Clips())
	{
		m_Analyzers[anim.Name] = new AnimationAnalyzer(behavier.Blocks());
		auto& analyzer = m_Analyzers[anim.Name];

		analyzer->PcaCutoff = CharacterPcaCutoff;
		analyzer->EnergyCutoff = CharacterEnergyCutoff;

		tasks.emplace_back(analyzer->ComputeFromFramesAsync(anim.GetFrameBuffer()));
	}
	
	when_all(tasks.begin(), tasks.end()).then([this]() {
		auto& blocks = m_pCharacter->Behavier().Blocks();
		for (auto& pair : m_Analyzers)
		{
			auto& key = pair.first;
			auto analyzer = pair.second;
			for (auto& bidx : analyzer->ActiveBlocks)
			{
				++blocks[bidx]->ActiveActionCount;
				blocks[bidx]->ActiveActions.emplace_back(key);
			}
		}
	});
}

struct SkeletonBlock
{
	DirectX::XMVECTOR Scale;
	DirectX::XMVECTOR CenterFromParentCenter;
	JointType SkeletonJoint;
	int ParentBlockIndex;
};

const SkeletonBlock g_SkeletonBlocks[] = {
	{ { 0.75f, 1.0f, 0.5f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f }, JointType_SpineMid, -1 },      //  0 - lower torso
	{ { 0.75f, 1.0f, 0.5f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_SpineShoulder, 0 },  //  1 - upper torso

	{ { 0.25f, 0.25f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_Neck, 1 },         //  2 - neck
	{ { 0.5f, 0.5f, 0.5f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_Head, 2 },            //  3 - head

	{ { 0.35f, 0.4f, 0.35f, 0.0f },{ 0.5f, 1.0f, 0.0f, 0.0f }, JointType_ShoulderLeft, 1 },  //  4 - Left shoulderblade
	{ { 0.25f, 1.0f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_ElbowLeft, 4 },     //  5 - Left upper arm
	{ { 0.15f, 1.0f, 0.15f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_WristLeft, 5 },     //  6 - Left forearm
	{ { 0.10f, 0.4f, 0.30f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_HandLeft, 6 },      //  7 - Left hand

	{ { 0.35f, 0.4f, 0.35f, 0.0f },{ -0.5f, 1.0f, 0.0f, 0.0f }, JointType_ShoulderRight, 1 },//   8 - Right shoulderblade
	{ { 0.25f, 1.0f, 0.25f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_ElbowRight, 8 },    //   9 - Right upper arm
	{ { 0.15f, 1.0f, 0.15f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_WristRight, 9 },    //  10 - Right forearm
	{ { 0.10f, 0.4f, 0.30f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_HandRight, 10 },    //  11 - Right hand

	{ { 0.35f, 0.4f, 0.35f, 0.0f },{ 0.5f, 0.0f, 0.0f, 0.0f }, JointType_HipLeft, 0 },       //  12 - Left hipblade
	{ { .4f, 1.0f, .4f, 0.0f },{ 0.5f, 0.0f, 0.0f, 0.0f }, JointType_KneeLeft, 12 },         //  13 - Left thigh
	{ { .3f, 1.0f, .3f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_AnkleLeft, 13 },        //  14 - Left calf
																							 //{ { .35f, .6f, .20f, 0.0f }, { 0.0f, 1.0f, 0.0f, 0.0f }, JointType_FootLeft , 14 },     //     - Left foot

	{ { 0.35f, 0.4f, 0.35f, 0.0f },{ -0.5f, 0.0f, 0.0f, 0.0f }, JointType_HipRight, 0 },     //  15  - Right hipblade
	{ { .4f, 1.0f, .4f, 0.0f },{ -0.5f, 0.0f, 0.0f, 0.0f }, JointType_KneeRight, 15 },       //  16  - Right thigh
	{ { .3f, 1.0f, .3f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f }, JointType_AnkleRight, 16 },       //  17  - Right calf
																							 //{ { .35f, .6f, .20f, 0.0f }, { 0.0f, 1.0f, 0.0f, 0.0f }, JointType_FootRight, 18 },     //      - Right foot

};

const UINT BLOCK_COUNT = _countof(g_SkeletonBlocks);
const UINT BODY_COUNT = 6;
const UINT JOINT_COUNT = 25;

