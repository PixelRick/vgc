// Copyright 2021 The VGC Developers
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/vgc/vgc/blob/master/COPYRIGHT
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vgc/graphics/engine.h>

namespace vgc {
namespace graphics {

Engine::Engine()
    : Object()
    , resourceList_(new detail::ResourceList())
{
    projectionMatrixStack_.emplaceLast(geometry::Mat4f::identity);
    viewMatrixStack_.emplaceLast(geometry::Mat4f::identity);
}

// XXX add try/catch ?
void Engine::renderThreadProc_() {
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    while (1) {
        lock.lock();

        // wait for work or stop request
        wakeRenderThreadConditionVariable_.wait(lock, [&]{ return !commandQueue_.isEmpty() || stopRequested_; });

        // if requested, stop
        if (stopRequested_) {
            // cancel submitted lists
            for (const CommandList& l : commandQueue_) {
                for (Resource* resource : l.garbagedResources) {
                    resource->release_(this);
                }
            }
            commandQueue_.clear();
            lastExecutedCommandListId_ = lastSubmittedCommandListId_;
            // release all resources..
            for (Resource* r : resourceList_->danglingResources_) {
                r->release_(this);
                delete r;
            }
            for (Resource* r : resourceList_->resources_) {
                r->release_(this);
                r->owningList_ = nullptr;
            }
            resourceList_->resources_.clear();
            resourceList_->danglingResources_.clear();
            // notify
            lock.unlock();
            renderThreadEventConditionVariable_.notify_all();
            return;
        }

        // else commandQueue_ is not empty, so prepare some work
        CommandList commandList = std::move(commandQueue_.first());
        commandQueue_.removeFirst();

        lock.unlock();

        // execute commands
        for (const CommandUPtr& command : commandList.commands) {
            command->execute(this);
        }

        lock.lock();
        ++lastExecutedCommandListId_;
        lock.unlock();

        renderThreadEventConditionVariable_.notify_all();

        // release garbaged resources
        for (Resource* r : commandList.garbagedResources) {
            r->release_(this);
            delete r;
        }
    }
}

void Engine::startRenderThread_() {
    if (stopRequested_) {
        throw core::LogicError("Engine: restarts are not supported.");
    }
    if (!running_) {
        renderThread_ = std::thread([=]{ this->renderThreadProc_(); });
        running_ = true;
    }
}

void Engine::stopRenderThread_() {
    pendingCommands_.clear();
    lastBoundSwapChain_.reset();
    if (running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        stopRequested_ = true;
        lock.unlock();
        wakeRenderThreadConditionVariable_.notify_all();
        VGC_CORE_ASSERT(renderThread_.joinable());
        renderThread_.join();
        running_ = false;
        return;
    }
}

UInt Engine::submitPendingCommandList_() {
    std::unique_lock<std::mutex> lock(mutex_);
    bool notifyRenderThread = commandQueue_.isEmpty();
    commandQueue_.emplaceLast(std::move(pendingCommands_), std::move(resourceList_->danglingResources_));
    pendingCommands_.clear();
    resourceList_->danglingResources_.clear();
    UInt id = ++lastSubmittedCommandListId_;
    lock.unlock();
    if (notifyRenderThread) {
        wakeRenderThreadConditionVariable_.notify_all();
    }
    return id;
}

void Engine::waitCommandListTranslationFinished_(UInt commandListId) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (commandListId == 0) {
        commandListId = lastSubmittedCommandListId_;
    }
    renderThreadEventConditionVariable_.wait(lock, [&]{ return lastExecutedCommandListId_ == commandListId; });
    lock.unlock();
}

} // namespace graphics
} // namespace vgc
