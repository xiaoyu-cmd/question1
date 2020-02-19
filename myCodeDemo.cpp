#ifndef __dragon__EmojiFactory__
#define __dragon__EmojiFactory__

#include "cocos2d.h"
#include "gif/CacheGif.h"

using namespace cocos2d;

BEGIN_NS_DRAGON

class CC_DLL EmojiFactory {
public:
    EmojiFactory();
    ~EmojiFactory();
public:
    typedef enum {
        FLASH,
        GIF,
        IMAGE,
    } EmojiType;

    typedef struct {
        EmojiType _type;
        std::string _identifier;
        std::string _source;
        cocos2d::Size _contentSize;
        Vec2 _anchorPoint;
    } Emoji;
public:
    static EmojiFactory* getInstance();
    void addDefinition(EmojiType type,
                            const std::string& identifier,
                            const std::string& source,
                            cocos2d::Size contentSize = cocos2d::Size::ZERO,
                            Vec2 anchorPoint = Vec2::ANCHOR_MIDDLE);
    Node* create(const std::string& identifier);
    void clearDefinitions();
private:
    std::unordered_map<std::string, Emoji> _emojiPackage;
};

END_NS_DRAGON

#endif // __dragon__EmojiFactory__

#include "movieclip/MovieClip.h"
#include "EmojiFactory.h"

BEGIN_NS_DRAGON

EmojiFactory::EmojiFactory()
{
}

EmojiFactory::~EmojiFactory()
{
}

static EmojiFactory* _instance = nullptr;
EmojiFactory* EmojiFactory::getInstance()
{
    if (! _instance) {
        _instance = new EmojiFactory();
    }
    return _instance;
}

void EmojiFactory::addDefinition(EmojiType type,
                                      const std::string& identifier,
                                      const std::string& source,
                                      cocos2d::Size contentSize,
                                      Vec2 anchorPoint)
{
    Emoji emoji;
    emoji._type = type;
    emoji._identifier = identifier;
    emoji._source = source;
    emoji._contentSize = contentSize;
    emoji._anchorPoint = anchorPoint;
    if (_emojiPackage.find(identifier) == _emojiPackage.end()) {
        _emojiPackage[identifier] = emoji;
    }
}

Node* EmojiFactory::create(const std::string& identifier)
{
    if (_emojiPackage.find(identifier) == _emojiPackage.end()) {
        return NULL;
    }
    const Emoji& emoji = _emojiPackage[identifier];
    if (emoji._type == EmojiType::GIF) {
        std::string fullPath = FileUtils::getInstance()->fullPathForFilename(emoji._source);
        auto gifNode = CacheGif::create(fullPath.c_str());
        gifNode->setAnchorPoint(Vec2(emoji._anchorPoint));
        return gifNode;
    }
    else if (emoji._type == EmojiType::FLASH) {
        Node* flashNode = Node::create();
        flashNode->setContentSize(emoji._contentSize);
        MovieClip* mc = MovieClip::create(emoji._source);
        mc->setPosition(Vec2(flashNode->getContentSize().width*emoji._anchorPoint.x,
                             flashNode->getContentSize().height*emoji._anchorPoint.y));
        flashNode->addChild(mc);
        return flashNode;
    }
    else if (emoji._type == EmojiType::IMAGE) {
        Sprite* spriteNode = Sprite::createWithSpriteFrameName(emoji._source);
        if (spriteNode == NULL) {
            spriteNode = Sprite::create(emoji._source);
        }
        if (spriteNode) {
            spriteNode->setAnchorPoint(Vec2(emoji._anchorPoint));
            spriteNode->setContentSize(emoji._contentSize);
            return spriteNode;
        }
    }
    return NULL;
}

void EmojiFactory::clearDefinitions()
{
    _emojiPackage.clear();
}

END_NS_DRAGON


#ifndef __dragon__MaskNode__
#define __dragon__MaskNode__

#include "dragon/prefix.h"
#include "cocos2d.h"

BEGIN_NS_DRAGON

using cocos2d::Node;
using cocos2d::Sprite;
using cocos2d::Mat4;
using cocos2d::Rect;
using cocos2d::Texture2D;
using cocos2d::SpriteFrame;
using cocos2d::Renderer;
using cocos2d::CustomCommand;

class MaskEndNode;
class MaskWorker;

/**
 * @addtogroup sprite_nodes
 * @{
 */
class CC_DLL MaskBeginNode : public Node
{
public:
    enum MaskType {
        STENCIL = 1,
        SCISSOR = 2,
    };
    
    static MaskBeginNode* create(MaskType type, Node* maskContent = NULL);
    
    virtual void setMaskContent(Node* mask);
    virtual Node* getMaskContent();
    
    virtual void addChild(Node *child, int zOrder, int tag) override;
    virtual void addChild(Node *child, int zOrder, const std::string &name) override;
    
    virtual void visit(Renderer *renderer, const Mat4 &transform, uint32_t flags) override;
    virtual void draw(Renderer *renderer, const Mat4 &transform, uint32_t flags) override;
    void endVisit(Renderer *renderer);
    MaskEndNode * getEndNode();
    
    void setIsEnabled(bool enabled);
    bool isEnabled() { return _isEnabled; }
    bool isInverted() const;
    void setInverted(bool inverted);

CC_CONSTRUCTOR_ACCESS:
    MaskBeginNode(MaskType type);
    virtual ~MaskBeginNode();

    virtual bool initWithMaskContent(Node* maskContent);
    
protected:
    MaskWorker* _maskWorker;
    Node* _maskContent;
    MaskEndNode * _endNode;
    bool _isEnabled;
    bool _hasBegan;
    uint32_t _storedFlags;
    
    CC_DISALLOW_COPY_AND_ASSIGN(MaskBeginNode);
};

class CC_DLL MaskEndNode : public Node
{
    friend class MaskBeginNode;
public:
    virtual void addChild(Node *child, int zOrder, int tag) override;
    virtual void addChild(Node *child, int zOrder, const std::string &name) override;
    virtual void visit(Renderer *renderer, const Mat4& parentTransform, uint32_t parentFlags) override;

private:
    MaskEndNode();
    
    bool initWithMaskBeginNode(MaskBeginNode* beginNode);
    
    MaskBeginNode * _mBeginNode;
};


//// mask sprite
class CC_DLL MaskSprite: public Sprite
{
public:
    static MaskSprite* create();
    static MaskSprite* create(const std::string& filename);
    static MaskSprite* create(const std::string& filename, const Rect& rect);
    static MaskSprite* createWithTexture(Texture2D *texture);
    static MaskSprite* createWithTexture(Texture2D *texture, const Rect& rect, bool rotated=false, float textureScale=1.0f);
    static MaskSprite* createWithSpriteFrame(SpriteFrame *spriteFrame);
    static MaskSprite* createWithSpriteFrameName(const std::string& spriteFrameName);
    
    virtual void setTexture(Texture2D *texture) override;
    virtual void draw(Renderer *renderer, const Mat4 &transform, uint32_t flags) override;
    
    float getAlphaThreshold() const { return m_alphaThreshold; }
    void setAlphaThreshold(float value) { m_alphaThreshold = value; }
    
CC_CONSTRUCTOR_ACCESS:
    MaskSprite(void);
    
private:
    float m_alphaThreshold;
};

END_NS_DRAGON

#endif // __dragon__MaskNode__

#include "MaskNode.h"
#include "base/CCStencilStateManager.hpp"

using namespace cocos2d;

BEGIN_NS_DRAGON

class MaskWorker
{
public:
    virtual ~MaskWorker() {}
    
    virtual void setInverted(bool inverted) = 0;
    virtual bool isInverted() const = 0;
    virtual void applyMask(Node* maskContent, Renderer *renderer, float globalZOrder, const Mat4 &transform, uint32_t flags) = 0;
    virtual void cancelMask(Node* maskContent, Renderer *renderer, float globalZOrder, const Mat4 &transform, uint32_t flags) = 0;
};

class StencilMaskWorker: public MaskWorker
{
public:
    virtual void setInverted(bool inverted) override
    {
        _stencilStateManager.setInverted(inverted);
    }
    
    virtual bool isInverted() const override
    {
        return _stencilStateManager.isInverted();
    }
    
    virtual void applyMask(Node* maskContent, Renderer *renderer, float globalZOrder, const Mat4 &transform, uint32_t flags) override
    {
        //Add group command
        _groupCommand.init(globalZOrder);
        renderer->addCommand(&_groupCommand);
        
        renderer->pushGroup(_groupCommand.getRenderQueueID());
        
        _beforeVisitCmd.init(globalZOrder);
        _beforeVisitCmd.func = CC_CALLBACK_0(StencilStateManager::onBeforeVisit, &_stencilStateManager);
        renderer->addCommand(&_beforeVisitCmd);
        
        if (maskContent) {
            maskContent->visit(renderer, transform, flags);
        }
        
        _afterDrawStencilCmd.init(globalZOrder);
        _afterDrawStencilCmd.func = CC_CALLBACK_0(StencilStateManager::onAfterDrawStencil, &_stencilStateManager);
        renderer->addCommand(&_afterDrawStencilCmd);
    }
    
    virtual void cancelMask(Node* maskContent, Renderer *renderer, float globalZOrder, const Mat4 &transform, uint32_t flags) override
    {
        _beforeRestoreStencilCmd.init(globalZOrder);
        _beforeRestoreStencilCmd.func = CC_CALLBACK_0(StencilStateManager::onBeforeRestoreStencil, &_stencilStateManager);
        renderer->addCommand(&_beforeRestoreStencilCmd);
        
        // restore the stencil buffer
        if (maskContent) {
            maskContent->visit(renderer, transform, flags);
        }
        
        _afterVisitCmd.init(globalZOrder);
        _afterVisitCmd.func = CC_CALLBACK_0(StencilStateManager::onAfterVisit, &_stencilStateManager);
        renderer->addCommand(&_afterVisitCmd);
        
        renderer->popGroup();
    }
    
private:
    cocos2d::StencilStateManager _stencilStateManager;
    cocos2d::GroupCommand _groupCommand;
    cocos2d::CustomCommand _beforeVisitCmd;
    cocos2d::CustomCommand _afterDrawStencilCmd;
    cocos2d::CustomCommand _beforeRestoreStencilCmd;
    cocos2d::CustomCommand _afterVisitCmd;
};


class ScissorMaskWorker: public MaskWorker
{
public:
    ~ScissorMaskWorker() {}
    
    virtual void setInverted(bool inverted) override
    {
        if (inverted) {
            CCLOG("MaskType::SCISSOR mask node DON'T support `inverted` mode.");
        }
    }
    
    virtual bool isInverted() const override
    {
        return false;
    }
    
    virtual void applyMask(Node* maskContent, Renderer *renderer, float globalZOrder, const Mat4 &transform, uint32_t flags) override
    {
        Size size = maskContent->getContentSize();
        Vec2 verts[4];
        verts[0] = maskContent->convertToWorldSpace(Vec2(0, 0));
        verts[1] = maskContent->convertToWorldSpace(Vec2(0, size.height));
        verts[2] = maskContent->convertToWorldSpace(Vec2(size.width, size.height));
        verts[3] = maskContent->convertToWorldSpace(Vec2(size.width, 0));
        float x0 = FLT_MAX, y0 = FLT_MAX, x1 = FLT_MIN, y1 = FLT_MIN;
        for (int i=0; i<4; i++) {
            const Vec2& pt = verts[i];
            if (pt.x < x0) { x0 = pt.x; }
            if (pt.x > x1) { x1 = pt.x; }
            if (pt.y < y0) { y0 = pt.y; }
            if (pt.y > y1) { y1 = pt.y; }
        }
        _scissorRect.origin.x = x0;
        _scissorRect.origin.y = y0;
        _scissorRect.size.width = x1-x0;
        _scissorRect.size.height = y1-y0;
        
        //Add group command
        _groupCommand.init(globalZOrder);
        renderer->addCommand(&_groupCommand);
        
        renderer->pushGroup(_groupCommand.getRenderQueueID());
        
        _beforeVisitCmd.init(globalZOrder);
        _beforeVisitCmd.func = [=] () {
            this->onBeforeVisit();
        };
        renderer->addCommand(&_beforeVisitCmd);
    }
    
    virtual void cancelMask(Node* maskContent, Renderer *renderer, float globalZOrder, const Mat4 &transform, uint32_t flags) override
    {
        _afterVisitCmd.init(globalZOrder);
        _afterVisitCmd.func = _beforeVisitCmd.func = [=] () {
            this->onAfterVisit();
        };
        renderer->addCommand(&_afterVisitCmd);
        
        renderer->popGroup();
    }
    
private:
    void onBeforeVisit()
    {
        auto glview = Director::getInstance()->getOpenGLView();
        if (glview->isScissorEnabled()) {
            _prevScissorRect = glview->getScissorRect();
            _scissorRect.intersectMerge(_prevScissorRect);
        } else {
            _prevScissorRect.origin.x = 0;
            _prevScissorRect.origin.y = 0;
            _prevScissorRect.size.width = -1;
            _prevScissorRect.size.height = -1;
            
            glEnable(GL_SCISSOR_TEST);
        }
        glview->setScissorInPoints(_scissorRect.origin.x,
                                   _scissorRect.origin.y,
                                   _scissorRect.size.width,
                                   _scissorRect.size.height);
    }
    
    void onAfterVisit()
    {
        auto glview = Director::getInstance()->getOpenGLView();
        if (_prevScissorRect.size.width>=0 && _prevScissorRect.size.height>=0) {
            glview->setScissorInPoints(_prevScissorRect.origin.x,
                                       _prevScissorRect.origin.y,
                                       _prevScissorRect.size.width,
                                       _prevScissorRect.size.height);
        } else {
            glDisable(GL_SCISSOR_TEST);
        }
    }
    
private:
    cocos2d::GroupCommand _groupCommand;
    cocos2d::CustomCommand _beforeVisitCmd;
    cocos2d::CustomCommand _afterVisitCmd;
    cocos2d::Rect _scissorRect;
    cocos2d::Rect _prevScissorRect;
};


MaskBeginNode* MaskBeginNode::create(MaskType type, Node* maskContent)
{
    MaskBeginNode *beginNode = new (std::nothrow) MaskBeginNode(type);
    if (beginNode && beginNode->initWithMaskContent(maskContent))
    {
        beginNode->autorelease();
        return beginNode;
    }
    CC_SAFE_DELETE(beginNode);
    return NULL;
}

bool MaskBeginNode::initWithMaskContent(Node* maskContent)
{
    setMaskContent(maskContent);
    return true;
}

MaskBeginNode::MaskBeginNode(MaskType type)
: _maskWorker(NULL)
, _maskContent(NULL)
, _endNode(NULL)
, _isEnabled(true)
, _hasBegan(false)
{
    if (type==MaskType::SCISSOR) {
        _maskWorker = new (std::nothrow) ScissorMaskWorker();
    } else {
        // type==MaskType::STENCIL or else
        _maskWorker = new (std::nothrow) StencilMaskWorker();
    }
}

MaskBeginNode::~MaskBeginNode()
{
    CC_SAFE_RELEASE(_endNode);
    // CC_SAFE_RELEASE(_maskContent);
    CC_SAFE_DELETE(_maskWorker);
}

void MaskBeginNode::setIsEnabled(bool enabled)
{
    _isEnabled = enabled;
}

void MaskBeginNode::setMaskContent(Node* mask)
{
    if (mask==_maskContent) {
        return;
    }
    if (_maskContent) {
        _maskContent->removeFromParent();
        _maskContent = NULL;
        // this->setContentSize(Size::ZERO);
    }
    _maskContent = mask;
    if (_maskContent) {
        Node::addChild(_maskContent, 0, 0);
        // this->setContentSize(_maskContent->getContentSize());
    }
}

Node* MaskBeginNode::getMaskContent()
{
    return _maskContent;
}

MaskEndNode* MaskBeginNode::getEndNode()
{
    if (!_endNode)
    {
        _endNode = new MaskEndNode();
        _endNode->initWithMaskBeginNode(this);
    }
    return _endNode;
}

void MaskBeginNode::visit(Renderer *renderer, const Mat4 &parentTransform, uint32_t parentFlags)
{
    if (!isVisible()) {
        return;
    }
    _hasBegan = true;
    
    uint32_t flags = processParentFlags(parentTransform, parentFlags);
    _storedFlags = flags;
    
    // IMPORTANT:
    // To ease the migration to v3.0, we still support the Mat4 stack,
    // but it is deprecated and your code should not rely on it
    Director* director = Director::getInstance();
    CCASSERT(NULL != director, "Director is null when setting matrix stack");
    director->pushMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
    director->loadMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW, _modelViewTransform);
    
    //Add group command
    _maskWorker->applyMask(_maskContent, renderer, _globalZOrder, _modelViewTransform, flags);
}

void MaskBeginNode::endVisit(Renderer *renderer)
{
    if (!_hasBegan) {
        return;
    }
    _hasBegan = false;
    _maskWorker->cancelMask(_maskContent, renderer, _globalZOrder, _modelViewTransform, _storedFlags);
    Director::getInstance()->popMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
}

// draw
void MaskBeginNode::draw(Renderer *renderer, const Mat4 &transform, uint32_t flags)
{
    // Do nothing
}

void MaskBeginNode::addChild(Node *child, int zOrder, int tag)
{
    CCASSERT(false, "Can not addChild to MaskBeginNode");
}

void MaskBeginNode::addChild(Node *child, int zOrder, const std::string &name)
{
    CCASSERT(false, "Can not addChild to MaskBeginNode");
}

bool MaskBeginNode::isInverted() const
{
    return _maskWorker!=NULL && _maskWorker->isInverted();
}

void MaskBeginNode::setInverted(bool inverted)
{
    if (_maskWorker) {
        _maskWorker->setInverted(inverted);
    }
}

//// MaskEndNode
MaskEndNode::MaskEndNode()
{
}

bool MaskEndNode::initWithMaskBeginNode(MaskBeginNode* beginNode)
{
    if (this->Node::init()) {
        _mBeginNode = beginNode;
        return true;
    }
    return false;
}

void MaskEndNode::addChild(Node *child, int zOrder, int tag)
{
    CCASSERT(false, "Can not addChild to MaskEndNode");
}

void MaskEndNode::addChild(Node *child, int zOrder, const std::string &name)
{
    CCASSERT(false, "Can not addChild to MaskEndNode");
}

void MaskEndNode::visit(Renderer *renderer, const Mat4& parentTransform, uint32_t parentFlags)
{
    this->_mBeginNode->endVisit(renderer);
}

//// MaskSprite
MaskSprite::MaskSprite(void)
: Sprite()
, m_alphaThreshold(0.5)
{
    
}

MaskSprite* MaskSprite::create()
{
    MaskSprite *sprite = new (std::nothrow) MaskSprite();
    if (sprite && sprite->init())
    {
        sprite->autorelease();
        return sprite;
    }
    CC_SAFE_DELETE(sprite);
    return NULL;
}

MaskSprite* MaskSprite::create(const std::string& filename)
{
    MaskSprite *sprite = new (std::nothrow) MaskSprite();
    if (sprite && sprite->initWithFile(filename))
    {
        sprite->autorelease();
        return sprite;
    }
    CC_SAFE_DELETE(sprite);
    return NULL;
}

MaskSprite* MaskSprite::create(const std::string& filename, const Rect& rect)
{
    MaskSprite *sprite = new (std::nothrow) MaskSprite();
    if (sprite && sprite->initWithFile(filename, rect))
    {
        sprite->autorelease();
        return sprite;
    }
    CC_SAFE_DELETE(sprite);
    return NULL;
}

MaskSprite* MaskSprite::createWithTexture(Texture2D *texture)
{
    MaskSprite *sprite = new (std::nothrow) MaskSprite();
    if (sprite && sprite->initWithTexture(texture))
    {
        sprite->autorelease();
        return sprite;
    }
    CC_SAFE_DELETE(sprite);
    return NULL;
}

MaskSprite* MaskSprite::createWithTexture(Texture2D *texture, const Rect& rect, bool rotated, float textureScale)
{
    MaskSprite *sprite = new (std::nothrow) MaskSprite();
    if (sprite && sprite->initWithTexture(texture, rect, rotated, textureScale))
    {
        sprite->autorelease();
        return sprite;
    }
    CC_SAFE_DELETE(sprite);
    return NULL;
}

MaskSprite* MaskSprite::createWithSpriteFrame(SpriteFrame *spriteFrame)
{
    MaskSprite *sprite = new (std::nothrow) MaskSprite();
    if (sprite && sprite->initWithSpriteFrame(spriteFrame))
    {
        sprite->autorelease();
        return sprite;
    }
    CC_SAFE_DELETE(sprite);
    return NULL;
}

MaskSprite* MaskSprite::createWithSpriteFrameName(const std::string& spriteFrameName)
{
    MaskSprite *sprite = new (std::nothrow) MaskSprite();
    if (sprite && sprite->initWithSpriteFrameName(spriteFrameName))
    {
        sprite->autorelease();
        return sprite;
    }
    CC_SAFE_DELETE(sprite);
    return NULL;
}

void MaskSprite::setTexture(Texture2D *texture)
{
    Sprite::setTexture(texture);
    setGLProgramState(GLProgramState::getOrCreateWithGLProgramName(GLProgram::SHADER_NAME_POSITION_TEXTURE_ALPHA_TEST_NO_MV, texture));
}

// draw
void MaskSprite::draw(Renderer *renderer, const Mat4 &transform, uint32_t flags)
{
#if CC_USE_CULLING
    // Don't calculate the culling if the transform was not updated
    auto visitingCamera = Camera::getVisitingCamera();
    auto defaultCamera = Camera::getDefaultCamera();
    if (visitingCamera == defaultCamera) {
        _insideBounds = ((flags & FLAGS_TRANSFORM_DIRTY) || visitingCamera->isViewProjectionUpdated()) ? renderer->checkVisibility(transform, _contentSize) : _insideBounds;
    }
    else
    {
        // XXX: this always return true since
        _insideBounds = renderer->checkVisibility(transform, _contentSize);
    }
    
    if(_insideBounds)
#endif
    {
        getGLProgramState()->setUniformFloat("u_alpha_value", m_alphaThreshold);
        _trianglesCommand.init(_globalZOrder, _texture, getGLProgramState(), _blendFunc, _polyInfo.triangles, transform, getConcatenatedColorMatrix(),flags);
        renderer->addCommand(&_trianglesCommand);
    }
}

END_NS_DRAGON


void TextureCache::runTextureGC(eGCOption opt)
{
    GCLOG("TextureCache: before running GC: total-bytes=%lu, texture-number=%lu", getTotalTextureBytes(), _texCount);
    
    uint32_t now = msecondsSinceStarted();
    
    float gcInterval = (now-_lastGCTime)*0.001;
    if (_averageGCInterval <= 0) {
        _averageGCInterval = gcInterval;
    } else {
        _averageGCInterval = _averageGCInterval*0.3 + gcInterval*0.7;
    }
    
    if (opt==eGCOption::kGCRemoveAllUnused) {
        removeUnusedTextures();
    } else {
        TexPairArray unusedTextures;
        float residentFactor = _residentFactor;
        float resetInterval = _resetInterval*1000;
        if (now > _lastResetTime + resetInterval) {
            float correction = 1 - (now-_lastResetTime-resetInterval)/(resetInterval*10);
            if (correction < 0.001f) { correction = 0.001f; }
            residentFactor = residentFactor/correction;
        }
        collectUnusedTextures(unusedTextures, _textures, [&](Texture2D* tex)->bool{
            TexCacheInfo* info = tex->getCacheInfo();
            return info==NULL || info->localAccessCnt==0 || info->reloadMetrics < residentFactor;
        });
        if (unusedTextures.size() > 0) {
            if (_discardStrategy!=NULL) {
                _discardStrategy->sortCandidateTextures(unusedTextures);
            }
            SpriteFrameCache* spriteFrameCache = SpriteFrameCache::getInstance();
            if (opt==eGCOption::kGCKeepInSafetyZone) {
                for (auto it=unusedTextures.begin(); it!=unusedTextures.end(); ++it) {
                    if (it->second->getReferenceCount() > 0) {
                        // Texture is referenced by SpriteFrameCache. Remove those sprite frames.
                        spriteFrameCache->removeSpriteFramesFromTexture(it->second);
                    }
                    GCLOG("TextureCache: removing texture: (%u bytes) %s", it->second->getMemoryBytes(), it->first.c_str());
                    removeTextureForKey(it->first);
                    if (getTotalTextureBytes() <= _memSafetyValue) {
                        break;
                    }
                }
            } else {
                GCLOG("TextureCache: Unkown texture cache GC option: %d", opt);
            }
        }
    }
    
    if (_averageMemSizeAfterGC==0) {
        _averageMemSizeAfterGC = getTotalTextureBytes();
    } else {
        _averageMemSizeAfterGC = _averageMemSizeAfterGC*0.5 + getTotalTextureBytes()*0.5;
    }
    
    _lastGCTime = now;
    
    GCLOG("TextureCache: after running GC: total-bytes=%lu, texture-number=%lu", getTotalTextureBytes(), _texCount);
}

#include "MovieClip.h"
#include "MCTimeline.h"
#include "../MaskNode.h"
#include "MCLibrary.h"

BEGIN_NS_DRAGON

MovieClip::MovieClip()
: _timeline(NULL)
, _def(NULL)
, _speed(1.0f)
, _currentFrameIndex(0)
, _currentRatio(0)
, _delegate(NULL)
, _isScriptEnabled(false)
{
    _cascadeColorEnabled = true;
    _cascadeOpacityEnabled = true;
    _timeline = new MCTimeline(this);
}

MovieClip::~MovieClip()
{
    CC_SAFE_RELEASE(_delegate);
    CC_SAFE_RELEASE(const_cast<MCSymbolDef*>(_def));
    if (_timeline) {
        _timeline->discard();
        _timeline->release();
    }
}

MovieClip* MovieClip::create(const std::string& name)
{
    if (name.empty()) {
        return NULL;
    }
    MCObjectDef* def = MCLibrary::getInstance()->getObjectDefinition(name);
    const MCSymbolDef* symbolDef = def!=NULL ? def->toSymbolDef() : NULL;
    if (symbolDef!=NULL) {
        return MovieClip::create(symbolDef);
    }
    return NULL;
}

MovieClip* MovieClip::create(const MCSymbolDef* def)
{
    if (def==NULL) {
        return NULL;
    }
    MovieClip* obj = new MovieClip();
    if (obj && obj->initWithDefinition(def)) {
        obj->autorelease();
        return obj;
    }
    CC_SAFE_RELEASE(obj);
    return NULL;
}

bool MovieClip::initWithDefinition(const MCSymbolDef* def)
{
    assert(_def==NULL);
    _def = def;
    if (_def==NULL) {
        return false;
    }
    const_cast<MCSymbolDef*>(_def)->retain();
    
    _timeline->setFrameInterval(1.0f/(getPlaySpeed()*getFPS()));
    
    gotoAndPlay(1);
    
    return true;
}

MCTimelineGroup* MovieClip::getTimelineGroup() const
{
    if (_timeline) {
        return _timeline->getTimelineGroup();
    } else {
        return NULL;
    }
}

void MovieClip::setTimelineGroup(MCTimelineGroup* group)
{
    if (_timeline) {
        _timeline->setTimelineGroup(group);
    }
    SubMovieClips::iterator it = _subMovieClips.begin();
    for (; it!=_subMovieClips.end(); ++it) {
        MovieClip* subMc = (*it);
        if (subMc) {
            subMc->setTimelineGroup(group);
        }
    }
}

int MovieClip::getCurrentFrame()
{
    return _timeline->getCurrentFrame();
}

int MovieClip::getTotalFrames()
{
    return _def!=NULL ? _def->getFrameCount() : 0;
}

int MovieClip::getFPS()
{
    return _def!=NULL ? _def->getFPS() : 24;
}

void MovieClip::setPlaySpeed(float val)
{
    if (_speed==val) {
        return;
    }
    _speed = val;
    _timeline->setFrameInterval(1.0f/(getPlaySpeed()*getFPS()));
}

int MovieClip::checkFrameIndex(int frame)
{
    if (frame < 1) {
        return 1;
    } else if (frame > getTotalFrames()) {
        return getTotalFrames();
    } else {
        return frame;
    }
}

void MovieClip::play()
{
    _timeline->play();
}

void MovieClip::stop()
{
    _timeline->stop();
}

void MovieClip::gotoAndPlay(int frame)
{
    int frameIndex = checkFrameIndex(frame);
    _timeline->gotoFrame(frameIndex);
    _timeline->play();
}

void MovieClip::gotoAndPlay(const std::string& label)
{
    int frame = getFrameIndexByLabel(label);
    if (frame > 0) {
        gotoAndPlay(frame);
    }
}

void MovieClip::gotoAndStop(int frame)
{
    int frameIndex = checkFrameIndex(frame);
    _timeline->gotoFrame(frameIndex);
    _timeline->stop();
}

void MovieClip::gotoAndStop(const std::string& label)
{
    int frame = getFrameIndexByLabel(label);
    if (frame > 0) {
        gotoAndStop(frame);
    }
}

bool MovieClip::isPlaying()
{
    return _timeline->isPlaying();
}

void MovieClip::setLoopAsEmbeddedGraphics(MCKeyframe::LoopOption loopOption, int frame)
{
    // map loopOption to embedType
    MCTimeline::EmbedType embedType = (MCTimeline::EmbedType)(1 + loopOption);
    _timeline->setEmbedProperty(embedType, frame);
}

void MovieClip::setDelegate(MovieClipDelegate* delegate)
{
    if (_delegate==delegate) {
        return;
    }
    if (_delegate) {
        _delegate->release();
    }
    _delegate = delegate;
}

size_t MovieClip::getLabelsAtFrame(int frameIndex, std::vector<std::string>* result) const
{
    size_t count = 0;
    const MCSymbolDef::LabelArray& labels = _def->getLabelsAtFrame(frameIndex);
    MCSymbolDef::LabelArray::const_iterator it = labels.begin();
    for (; it!=labels.end(); ++it) {
        if (result) {
            result->push_back(*it);
        }
        count++;
    }
    return count;
}

size_t MovieClip::getFrameIndicesByLabel(const std::string& label, std::vector<int>* result) const
{
    size_t count = 0;
    _def->selectFrameByLabel(label,
                             [&](int x)->bool {
                                 if (result) {
                                     result->push_back(x);
                                 }
                                 count++;
                                 return true;
                             });
    return count;
}

int MovieClip::getFrameIndexByLabel(const std::string& label)
{
    union {
        unsigned int min;
        int frame;
    } v;
    v.frame = -1;
    _def->selectFrameByLabel(label,
                             [&](int x)->bool {
                                 if (((unsigned int)x) < v.min) {
                                     v.min = x;
                                 }
                                 return true;
                             });
    return v.frame;
}

cocos2d::Node* MovieClip::getMCObject(const MCLayerInfo* layerDef, const MCObjectDef* def)
{
    if (layerDef==NULL || def==NULL) {
        return NULL;
    }
    int layerIndex = layerDef->getLayerIndex();
    struct cache_key_t key = {
        layerIndex,
        def->getDefId()
    };
    
    InstanceCache::iterator it = _instancesCache.find(key);
    if (it != _instancesCache.end()) {
        return it->second;
    }
    
    int mask = layerDef->getMaskType();
    if (mask!=0) {
        MaskBeginNode* maskBegin = NULL;
        if (mask==2) {
            maskBegin = MaskBeginNode::create(MaskBeginNode::MaskType::SCISSOR);
        } else {
            maskBegin = MaskBeginNode::create(MaskBeginNode::MaskType::STENCIL);
        }
        MaskEndNode* maskEnd = maskBegin->getEndNode();
        int maskDepth = layerDef->getMaskDepth();
        assert(maskDepth>=0);
        addChild(maskEnd, -(layerIndex<<1));
        addChild(maskBegin, -((layerIndex+maskDepth)<<1) - 1);
        _instancesCache[key] = maskBegin;
        
        Node* inst = def->newInstance();
        if (dynamic_cast<MovieClip*>(inst) != NULL) {
            _subMovieClips.push_back(static_cast<MovieClip*>(inst));
        }
        if (inst!=NULL) {
            maskBegin->setIsEnabled(true);
            maskBegin->setMaskContent(inst);
            maskBegin->setContentSize(inst->getContentSize());
            maskBegin->setAnchorPoint(inst->getAnchorPoint());
        } else {
            maskBegin->setIsEnabled(false);
        }
        
        return maskBegin;
    } else {
        cocos2d::Node* inst = def->newInstance();
        if (dynamic_cast<MovieClip*>(inst) != NULL) {
            _subMovieClips.push_back(static_cast<MovieClip*>(inst));
        }
        if (inst!=NULL) {
            _instancesCache[key] = inst;
            inst->setCascadeColorEnabled(true);
            inst->setCascadeOpacityEnabled(true);
            addChild(inst, -(layerIndex<<1));
        }
        return inst;
    }
}

void MovieClip::resetMCObject(cocos2d::Node* node)
{
    assert(node!=NULL);
    node->setVisible(false);
    node->setName("");
}

bool MovieClip::updateFrame(int frame, int ratio)
{
    if (!_def) {
        return false;
    }
    
    int frameIndex = frame;
    int totalFrames = getTotalFrames();
    if (totalFrames <= 0) { return false; }
    while (frameIndex <= 0) { frameIndex += totalFrames; }
    while (frameIndex > totalFrames) { frameIndex -= totalFrames; }
    
    assert(ratio>=0 && ratio<100);
    assert(frameIndex>0 && frameIndex<=getTotalFrames());
    
    int ratioDiff = ratio-_currentRatio;
    const int ratioTolerance = 5;
    if (ratioDiff>-ratioTolerance && ratioDiff<ratioTolerance
        && frameIndex == _currentFrameIndex)
    {
        return false;
    }
    _currentFrameIndex = frameIndex;
    _currentRatio = ratio;
    
    const MCSymbolDef::LayerArray& layers = _def->getLayers();
    MCSymbolDef::LayerArray::const_iterator it = layers.begin();
    for (; it != layers.end(); ++it) {
        updateLayer(*it, frameIndex, ratio);
    }
    
    return true;
}

void MovieClip::updateLayer(const MCLayerInfo* layerDef, int frame, int ratio)
{
    if (!layerDef) {
        return;
    }
    
    int layerIndex = layerDef->getLayerIndex();
    struct inst_info inst = {NULL, NULL};
    LayerInstances::iterator it0 = _activeInstances.find(layerIndex);
    if (it0 != _activeInstances.end()) {
        inst = it0->second;
    }
    
    const MCKeyframe *f1, *f2;
    f1 = layerDef->getKeyframe(frame, &f2);
    if (f1 == NULL) {
        if (inst.node) {
            resetMCObject(inst.node);
            _activeInstances.erase(layerIndex);
        }
        return;
    }
    
    const MCObjectDef* instDef = f1->getObjectDef();
    if (instDef == NULL && inst.node != NULL) {
        resetMCObject(inst.node);
        _activeInstances.erase(layerIndex);
        return;
    }
    
    Node* newNode = getMCObject(layerDef, instDef);
    if (newNode != inst.node) {
        if (inst.node) {
            resetMCObject(inst.node);
        }
        MovieClip* mc = dynamic_cast<MovieClip*>(newNode);
        if (mc) {
            mc->setTimelineGroup(this->getTimelineGroup());
            updateTimelineRelation(mc, f1->isSharingParentTimeline());
        }
    }
    
    if (newNode != inst.node || f1 != inst.keyframe) {
        struct inst_info tmp {newNode, f1};
        _activeInstances[layerIndex] = tmp;
    }
    
    // update newInst's properties
    float p = 0.0f;
    if (f2) {
        int i1 = f1->frameIndex;
        int i2 = f2->frameIndex;
        assert(i2 > i1);
        p = (frame + ratio*0.01f - i1) / (i2-i1);
    }
    f1->updateObject(newNode, p, f2, inst.keyframe != f1);
}

void MovieClip::updateTimelineRelation(MovieClip* child, bool isSharingTimeline)
{
    MCTimeline* tl = child->getTimeline();
    MCTimeline* parent = tl->getParent();
    if (isSharingTimeline) {
        if (parent != getTimeline()) {
            if (parent) {
                parent->removeChild(tl);
            }
            getTimeline()->addChild(tl);
        }
    } else {
        if (parent) {
            parent->removeChild(tl);
        }
    }
}

/// movieclip events
void MovieClip::_didEnterFrame(int frameIndex)
{
    // printf("%p: ENTER FRAME: %d\n", this, frameIndex);
    if (_isScriptEnabled) {
        _runScript(frameIndex, false);
    }
    if (_delegate) {
        _delegate->mcDidEnterFrame(this, frameIndex);
    }
}

void MovieClip::_willLeaveFrame(int frameIndex)
{
    // printf("%p: LEAVE FRAME: %d\n", this, frameIndex);
    if (_delegate) {
        _delegate->mcWillLeaveFrame(this, frameIndex);
    }
    if (_isScriptEnabled) {
        _runScript(frameIndex, true);
    }
    if (frameIndex == getTotalFrames()) {
        if (_delegate) {
            _delegate->mcDidReachEnding(this);
        }
    }
}

void MovieClip::_didUpdateFrame(int frameIndex, int ratio)
{
    // printf("%p: UPDATE FRAME: %d\n", this, frameIndex);
    assert(ratio>=0 && ratio<100);
    float frame = frameIndex + ratio*0.01f;
    if (_delegate) {
        _delegate->mcDidUpdateFrame(this, frame);
    }
}

void MovieClip::_runScript(int frameIndex, bool isLeavingFrame)
{
    if (_delegate==NULL) {
        return;
    }
    const MCSymbolDef::ScriptArray* scripts = _def->getScriptsAtFrame(frameIndex);
    if (scripts==NULL) {
        return;
    }
    MCSymbolDef::ScriptArray::const_iterator it = scripts->begin();
    for (; it != scripts->end(); ++it) {
        if (((it->options & MCSymbolDef::kTriggerWhenLeave) != 0) == isLeavingFrame) {
            MCScriptInfo script = {
                it->lang.c_str(),
                it->source.c_str(),
            };
            _delegate->mcRunScript(this, script);
        }
    }
}

END_NS_DRAGON
