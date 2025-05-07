
#include <algorithm>
#include <cstring>
#include <deque>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

template <class Iterator>
class IteratorRange {
public:
    IteratorRange(Iterator begin, Iterator end) : begin_(begin), end_(end) {}
    Iterator begin() const { return begin_; }
    Iterator end() const { return end_; }
private:
    Iterator begin_, end_;
};

namespace traverses {

template <class Vertex, class Graph, class Visitor>
void BreadthFirstSearch(Vertex origin_vertex, const Graph &graph, Visitor visitor) {
    std::queue<Vertex> queue;
    std::unordered_set<Vertex> discovered;

    queue.push(origin_vertex);
    discovered.insert(origin_vertex);
    visitor.DiscoverVertex(origin_vertex);

    while (!queue.empty()) {
        Vertex cur = queue.front();
        queue.pop();
        visitor.ExamineVertex(cur);

        for (const auto &edge : OutgoingEdges(graph, cur)) {
            visitor.ExamineEdge(edge);
            Vertex target = GetTarget(graph, edge);
            if (discovered.find(target) == discovered.end()) {
                discovered.insert(target);
                visitor.DiscoverVertex(target);
                queue.push(target);
            }
        }
    }
}

template <class Vertex, class Edge>
class BfsVisitor {
public:
    virtual void DiscoverVertex(Vertex) {}
    virtual void ExamineEdge(const Edge&) {}
    virtual void ExamineVertex(Vertex) {}
    virtual ~BfsVisitor() = default;
};

}

namespace aho_corasick {

struct AutomatonNode {
    AutomatonNode() : suffix_link(nullptr), terminal_link(nullptr) {}
    std::vector<size_t> terminated_string_ids;
    std::map<char, AutomatonNode> trie_transitions;
    std::map<char, AutomatonNode*> automaton_transitions;
    AutomatonNode* suffix_link;
    AutomatonNode* terminal_link;
};

AutomatonNode* GetTrieTransition(AutomatonNode* node, char character) {
    auto it = node->trie_transitions.find(character);
    return it != node->trie_transitions.end() ? &it->second : nullptr;
}

AutomatonNode* GetAutomatonTransition(AutomatonNode* node, AutomatonNode* root, char character) {
    auto& cache = node->automaton_transitions;
    auto it = cache.find(character);
    if (it != cache.end()) {
        return it->second;
    }
    if (AutomatonNode* transition = GetTrieTransition(node, character)) {
        cache[character] = transition;
        return transition;
    }
    if (node == root) {
        cache[character] = root;
        return root;
    }
    AutomatonNode* suffix_transition = GetAutomatonTransition(node->suffix_link, root, character);
    cache[character] = suffix_transition;
    return suffix_transition;
}

namespace internal {

class AutomatonGraph {
public:
    struct Edge {
        Edge(AutomatonNode* source, AutomatonNode* target, char character)
            : source(source), target(target), character(character) {}
        AutomatonNode* source;
        AutomatonNode* target;
        char character;
    };
};

std::vector<typename AutomatonGraph::Edge> OutgoingEdges(const AutomatonGraph&, AutomatonNode* vertex) {
    std::vector<typename AutomatonGraph::Edge> edges;
    for (auto& transition : vertex->trie_transitions) {
        edges.emplace_back(vertex, &transition.second, transition.first);
    }
    return edges;
}

AutomatonNode* GetTarget(const AutomatonGraph&, const AutomatonGraph::Edge& edge) {
    return edge.target;
}

class SuffixLinkCalculator : public traverses::BfsVisitor<AutomatonNode*, AutomatonGraph::Edge> {
public:
    explicit SuffixLinkCalculator(AutomatonNode* root) : root_(root) {}
    void ExamineVertex(AutomatonNode* node) override {
        if (!node->suffix_link) {
            node->suffix_link = root_;
        }
    }
    void ExamineEdge(const AutomatonGraph::Edge& edge) override {
        edge.target->suffix_link = edge.source == root_ ? root_ : 
            GetAutomatonTransition(edge.source->suffix_link, root_, edge.character);
    }
private:
    AutomatonNode* root_;
};

class TerminalLinkCalculator : public traverses::BfsVisitor<AutomatonNode*, AutomatonGraph::Edge> {
public:
    explicit TerminalLinkCalculator(AutomatonNode* root) : root_(root) {}
    void DiscoverVertex(AutomatonNode* node) override {
        if (node == root_) {
            node->terminal_link = nullptr;
            return;
        }
        if (!node->suffix_link->terminated_string_ids.empty()) {
            node->terminal_link = node->suffix_link;
        } else {
            node->terminal_link = node->suffix_link->terminal_link;
        }
    }
private:
    AutomatonNode* root_;
};

}

class NodeReference {
public:
    NodeReference() : node_(nullptr), root_(nullptr) {}
    NodeReference(AutomatonNode* node, AutomatonNode* root) : node_(node), root_(root) {}

    NodeReference Next(char character) const {
        return NodeReference(GetAutomatonTransition(node_, root_, character), root_);
    }

    template <class Callback>
    void GenerateMatches(Callback on_match) const {
        for (NodeReference node = *this; node; node = node.TerminalLink()) {
            for (size_t id : node.TerminatedStringIds()) {
                on_match(id);
            }
        }
    }

    bool IsTerminal() const {
        return !node_->terminated_string_ids.empty();
    }

    explicit operator bool() const { return node_ != nullptr; }
    bool operator==(NodeReference other) const {
        return node_ == other.node_ && root_ == other.root_;
    }

private:
    using TerminatedStringIterator = std::vector<size_t>::const_iterator;
    using TerminatedStringIteratorRange = IteratorRange<TerminatedStringIterator>;

    NodeReference TerminalLink() const {
        return NodeReference(node_->terminal_link, root_);
    }

    TerminatedStringIteratorRange TerminatedStringIds() const {
        return TerminatedStringIteratorRange(
            node_->terminated_string_ids.begin(),
            node_->terminated_string_ids.end());
    }

    AutomatonNode* node_;
    AutomatonNode* root_;
};

class AutomatonBuilder;

class Automaton {
public:
    Automaton() = default;
    Automaton(const Automaton&) = delete;
    Automaton& operator=(const Automaton&) = delete;

    NodeReference Root() {
        return NodeReference(&root_, &root_);
    }

private:
    AutomatonNode root_;
    friend class AutomatonBuilder;
};

class AutomatonBuilder {
public:
    void Add(const std::string& string, size_t id) {
        words_.push_back(string);
        ids_.push_back(id);
    }

    std::unique_ptr<Automaton> Build() {
        auto automaton = std::make_unique<Automaton>();
        BuildTrie(words_, ids_, automaton.get());
        BuildSuffixLinks(automaton.get());
        BuildTerminalLinks(automaton.get());
        return automaton;
    }

private:
    static void BuildTrie(const std::vector<std::string>& words, const std::vector<size_t>& ids, Automaton* automaton) {
        for (size_t i = 0; i < words.size(); ++i) {
            AddString(&automaton->root_, ids[i], words[i]);
        }
    }

    static void AddString(AutomatonNode* root, size_t string_id, const std::string& string) {
        AutomatonNode* cur = root;
        for (char ch : string) {
            cur = &cur->trie_transitions[ch];
        }
        cur->terminated_string_ids.push_back(string_id);
    }

    static void BuildSuffixLinks(Automaton* automaton) {
        internal::SuffixLinkCalculator visitor(&automaton->root_);
        traverses::BreadthFirstSearch(&automaton->root_, internal::AutomatonGraph(), visitor);
    }

    static void BuildTerminalLinks(Automaton* automaton) {
        internal::TerminalLinkCalculator visitor(&automaton->root_);
        traverses::BreadthFirstSearch(&automaton->root_, internal::AutomatonGraph(), visitor);
    }

    std::vector<std::string> words_;
    std::vector<size_t> ids_;
};

}

template <class Predicate>
std::vector<std::string> Split(const std::string& string, Predicate is_delimiter) {
    std::vector<std::string> result;
    size_t start = 0;
    size_t end = 0;
    while (end <= string.size()) {
        if (end == string.size() || is_delimiter(string[end])) {
            result.push_back(string.substr(start, end - start));
            start = end + 1;
        }
        end++;
    }
    return result;
}

class WildcardMatcher {
public:
    WildcardMatcher() : number_of_words_(0), pattern_length_(0) {}

    static WildcardMatcher BuildFor(const std::string &pattern, char wildcard) {
        WildcardMatcher matcher;
        matcher.pattern_length_ = pattern.size();
        auto is_wildcard = [wildcard](char ch) { return ch == wildcard; };
        std::vector<std::string> parts = Split(pattern, is_wildcard);

        std::vector<std::pair<std::string, size_t>> words_with_positions;
        size_t pos = 0;
        size_t co = 0;

        aho_corasick::AutomatonBuilder builder;

        for (const auto& part : parts) {
            if (!part.empty()) {
                matcher.pos_in_pattern_.push_back((pos += part.size()) - 1);
                builder.Add(part, co);
                ++co;
            }
            ++pos;
        }
        
        matcher.number_of_words_ = matcher.pos_in_pattern_.size();

        matcher.aho_corasick_automaton_ = builder.Build();
        matcher.state_ = matcher.aho_corasick_automaton_->Root();
        
        matcher.words_occurrences_by_position_.assign(matcher.pattern_length_ + 1, 0);
        matcher.offset_ = 0;
        
        return matcher;
    }

    void Reset() {
        state_ = aho_corasick_automaton_->Root();
        std::fill(words_occurrences_by_position_.begin(), words_occurrences_by_position_.end(), 0);
        offset_ = 0;
    }

    template <class Callback>
    void Scan(char character, Callback on_match) {
        ++offset_;
        words_occurrences_by_position_.pop_front();
        words_occurrences_by_position_.push_back(0);
        state_ = state_.Next(character);
        state_.GenerateMatches([this](size_t word_id) {
            if (word_id < number_of_words_) {
                size_t delta = pattern_length_ - pos_in_pattern_[word_id] - 1;
                ++words_occurrences_by_position_[delta];
            }
        });
        if (words_occurrences_by_position_[0] == number_of_words_ && offset_ >= pattern_length_) {
            on_match();
        }
    }

private:
    std::deque<size_t> words_occurrences_by_position_;
    std::vector<size_t> pos_in_pattern_;
    aho_corasick::NodeReference state_;
    size_t number_of_words_;
    size_t pattern_length_;
    std::unique_ptr<aho_corasick::Automaton> aho_corasick_automaton_;
    size_t offset_;
};

std::string ReadString(std::istream& input_stream) {
    std::string result;
    std::getline(input_stream, result);
    return result;
}

std::vector<size_t> FindFuzzyMatches(const std::string& pattern_with_wildcards, const std::string& text, char wildcard) {
    WildcardMatcher matcher = WildcardMatcher::BuildFor(pattern_with_wildcards, wildcard);
    std::vector<size_t> matches;
    for (size_t i = 0; i < text.size(); ++i) {
        matcher.Scan(text[i], [&matches, i, pattern_length = pattern_with_wildcards.size()]() {
            matches.push_back(i + 1 - pattern_length);
        });
    }
    return matches;
}

void Print(const std::vector<size_t>& sequence) {
    std::cout << sequence.size() << "\n";
    for (auto i : sequence) {
        std::cout << i << '\n';
    }
}
