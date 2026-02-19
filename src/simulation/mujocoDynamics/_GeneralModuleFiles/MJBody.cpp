/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "MJBody.h"
#include "MJScene.h"
#include "MJSpec.h"

#include <stdexcept>
#include <type_traits>
#include <unordered_map>

#include <iostream>

namespace
{
    /** Returns true if the first three scalar joints are translational
     * and along the main axis ([1,0,0], [0,1,0], [0,0,1]).
    */
    bool areJoints3DTranslation(std::list<MJScalarJoint>& joints)
    {
        if (joints.size() < 3) return false;

        size_t idx = 0;
        for (auto&& joint : joints)
        {
            if (idx == 3) break;
            if (joint.isHinge()) return false;
            if (std::fabs(joint.getAxis()[idx] - 1) > 1e-10) return false;
            ++idx;
        }

        return true;
    }
}

MJBody::MJBody(mjsBody* body, MJSpec& spec)
    : MJObject(body), spec(spec)
{

    // SITES
    for (auto child = mjs_firstChild(body, mjOBJ_SITE, 0); child; child = mjs_nextChild(body, child, 0))
    {

        auto mjssite = mjs_asSite(child);
        assert(mjssite != NULL);
        this->sites.emplace_back(mjssite, *this);
    }

    if (!this->hasSite(this->name + "_com")) {
        this->addSite(this->name + "_com", Eigen::Vector3d::Zero());
    }

    if (!this->hasSite(this->name + "_origin")) {
        this->addSite(this->name + "_origin", Eigen::Vector3d::Zero());
    }

    // JOINTS
    for (auto child = mjs_firstChild(body, mjOBJ_JOINT, 0); child; child = mjs_nextChild(body, child, 0))
    {

        auto mjsjoint = mjs_asJoint(child);
        assert(mjsjoint != NULL);

        switch (mjsjoint->type)
        {
        case mjJNT_HINGE:
        case mjJNT_SLIDE:
            this->scalarJoints.emplace_back(mjsjoint, *this);
            break;
        case mjJNT_BALL:
            this->ballJoint.emplace(mjsjoint, *this);
            break;
        case mjJNT_FREE:
            this->freeJoint.emplace(mjsjoint, *this);
            break;
        default:
            throw std::runtime_error("Unknown joint type."); // should not happen unless MuJoCo adds new joint
        }
    }

}

void MJBody::configure(const mjModel* mujocoModel)
{
    MJObject::configure(mujocoModel);

    for (auto&& joint : this->scalarJoints) {
        joint.configure(mujocoModel);
    }
    if (this->ballJoint.has_value()) {
        this->ballJoint->configure(mujocoModel);
    }
    if (this->freeJoint.has_value()) {
        this->freeJoint->configure(mujocoModel);
    }

    for (auto&& site : this->sites) {
        site.configure(mujocoModel);
    }

    // Update the position of the center of mass
    auto& com = this->getCenterOfMass();
    auto bodyId = this->getId();
    auto siteId = com.getId();
    std::copy_n(mujocoModel->body_ipos + 3 * bodyId, 3, mujocoModel->site_pos + 3 * siteId);

    // Update the mass property states
    if (!this->massState) {
        // Should not happen
        this->getSpec().getScene().logAndThrow("Tried to configure MJBody before massState was created.");
    }

    // Use subtree mass to account for the mass of this body and all its children.
    this->massState->setState(Eigen::Matrix<double, 1, 1>{mujocoModel->body_subtreemass[this->getId()]});
}

MJSite& MJBody::getSite(const std::string& name)
{
    auto sitePtr = std::find_if(std::begin(sites), std::end(sites), [&](auto&& obj) {
        return obj.getName() == name;
    });

    if (sitePtr == std::end(sites)) {
        this->getSpec().getScene().logAndThrow("Unknown site '" + name + "' in body '" + this->name + "'");
    }

    return *sitePtr;
}

MJScalarJoint& MJBody::getScalarJoint(const std::string& name)
{
    auto jointPtr = std::find_if(std::begin(scalarJoints), std::end(scalarJoints), [&](auto&& obj) {
        return obj.getName() == name;
    });

    if (jointPtr != std::end(scalarJoints)) return *jointPtr;

    this->getSpec().getScene().logAndThrow("Unknown scalar joint '" + name + "' in body '" + this->getName() + "'");
}

MJBallJoint& MJBody::getBallJoint()
{
    if (!this->ballJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to get a ball joint for a body without ball joints: " +
                                                    name);
    }
    return this->ballJoint.value();
}


MJFreeJoint & MJBody::getFreeJoint()
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to get a free joint for a body without free joints: " +
                                                    name);
    }
    return this->freeJoint.value();
}

void MJBody::setPosition(const Eigen::Vector3d& position)
{
    if (this->freeJoint.has_value()) {
        this->freeJoint.value().setPosition(position);
    } else if (areJoints3DTranslation(scalarJoints))
    {
        size_t idx = 0;
        for (auto&& joint : scalarJoints)
        {
            if (idx == 3) break;
            joint.setPosition(position[idx]);
            ++idx;
        }
    } else {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to set position in a body with no 'free' joint or no 3D translational joints " +
                                                    name);
    }
}

void MJBody::setVelocity(const Eigen::Vector3d& velocity)
{
    if (this->freeJoint.has_value()) {
        this->freeJoint.value().setVelocity(velocity);
    } else if (areJoints3DTranslation(scalarJoints))
    {
        size_t idx = 0;
        for (auto&& joint : scalarJoints)
        {
            if (idx == 3) break;
            joint.setVelocity(velocity[idx]);
            ++idx;
        }
    } else {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to set velocity in a body with no 'free' joint or no 3D translational joints " +
                                                    name);
    }
}

void MJBody::setAttitude(const Eigen::MRPd& attitude)
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to set attitude in non-free body " +
                                                    name);
    }
    this->freeJoint.value().setAttitude(attitude);
}

void MJBody::setAttitudeRate(const Eigen::Vector3d& attitudeRate)
{
    if (!this->freeJoint.has_value()) {
        this->getSpec().getScene().logAndThrow<std::runtime_error>("Tried to set attitude rate in non-free body " +
                                                    name);
    }
    this->freeJoint.value().setAttitudeRate(attitudeRate);
}

void MJBody::writeFwdKinematicsMessages(mjModel* m, mjData* d, uint64_t CurrentSimNanos)
{
    for (auto&& site : this->sites) {
        site.writeFwdKinematicsMessage(m, d, CurrentSimNanos);
    }
}

void MJBody::writeStateDependentOutputMessages(uint64_t CurrentSimNanos)
{
    SCMassPropsMsgPayload massPropertiesOutMsgPayload;

    massPropertiesOutMsgPayload.massSC = this->massState->getState()(0);
    this->massPropertiesOutMsg.write(&massPropertiesOutMsgPayload,
                                     this->getSpec().getScene().moduleID,
                                     CurrentSimNanos);

    for (auto&& joint : this->scalarJoints) {
        joint.writeJointStateMessage(CurrentSimNanos);
    }
}

void MJBody::registerStates(DynParamRegisterer paramManager)
{
    this->massState = paramManager.registerState(1, 1, "mass");
}

bool MJBody::updateMujocoModelFromMassProps()
{
    auto m = spec.getMujocoModel();
    const int id = this->getId();

    // Optional: skip wheels if you want
    const char* bname = mj_id2name(m, mjOBJ_BODY, id);
    if (bname && std::strstr(bname, "wheel") != nullptr) {
        return false;
    }

    double newMass = this->massState->getState()(0, 0);
    if (!std::isfinite(newMass) || newMass < 1e-6) newMass = 1e-6;

    // Use current model mass only as reference for "has it changed?"
    const double oldMass = m->body_mass[id];
    if (std::abs(newMass - oldMass) <= 1e-12 * std::max(1.0, oldMass)) {
        return false;
    }

    // Scale inertia consistently (use model inertia as baseline; OK for minimal change)
    const double scale = newMass / std::max(oldMass, 1e-6);

    // Write into mjSpec object (mjsBody), NOT mjModel
    this->mjsObject->mass = newMass;

    constexpr double kMinInertia = 1e-7;
    for (int i = 0; i < 3; ++i) {
        double I = this->mjsObject->inertia[i] * scale;
        if (!std::isfinite(I) || I < kMinInertia) I = kMinInertia;
        this->mjsObject->inertia[i] = I;
    }

    // Tell the scene/spec it must recompile before continuing
    this->getSpec().requestRecompile();   // you may need to add this helper; see below
    return true;
}


void MJBody::updateMassPropsDerivative()
{
    if (this->derivativeMassPropertiesInMsg.isLinked()) {
        auto deriv = this->derivativeMassPropertiesInMsg();
        double dm_sc = deriv.massSC;
        auto msstate = this->massState->getState()(0, 0);
        constexpr double kMinMass = 1e-6;
        if (msstate <= kMinMass && dm_sc < 0.0) dm_sc = 0.0;
        
        this->massState->setDerivative(Eigen::Matrix<double, 1, 1>{deriv.massSC});
    }
}

void MJBody::updateConstrainedEqualityJoints()
{
    for (auto&& joint : this->scalarJoints) {
        joint.updateConstrainedEquality();
    }
}

void MJBody::addSite(std::string name, const Eigen::Vector3d& position, const Eigen::MRPd& attitude)
{

    if (this->hasSite(name)) {
        this->getSpec().getScene().logAndThrow("Tried to create site " + name + " twice for body " +
                                     this->name);
    }

    auto mjssite = mjs_addSite(this->mjsObject, 0);
    mjs_setString(mjssite->name, name.c_str());

    auto& site = this->sites.emplace_back(mjssite, *this);

    spec.markAsNeedingToRecompileModel(); // Any updates to the 'structure' (i.e. new elements), we need to recompile

    site.setPositionRelativeToBody(position);
    site.setAttitudeRelativeToBody(attitude);
}

bool MJBody::hasSite(const std::string& name) const
{
    return std::find_if(std::begin(sites), std::end(sites), [&](auto&& obj) {
               return obj.getName() == name;
           }) != std::end(sites);
}

const Eigen::Vector3d MJBody::getCurrentSubtreeCenterOfMass() {
    mjData* mjdata = this->getSpec().getMujocoData();
    const double* origin = mjdata->xpos + 3 * this->getId();
    const double* com = mjdata->subtree_com + 3 * this->getId();
    const double* rot = mjdata->xmat + 9 * this->getId();

    Eigen::Vector3d t = Eigen::Map<const Eigen::Vector3d>(com) 
                      - Eigen::Map<const Eigen::Vector3d>(origin);
    Eigen::Matrix3d R = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(rot);

    return R.transpose() * t;
}
