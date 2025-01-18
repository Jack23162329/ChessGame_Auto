# chess_model_server.py
from flask import Flask, request, jsonify
from chess_zero.agent.player_chess import ChessPlayer
from chess_zero.config import Config, PlayWithHumanConfig
from chess_zero.env.chess_env import ChessEnv

app = Flask(__name__)

# Initialize chess components
def get_player(config):
    from chess_zero.agent.model_chess import ChessModel
    from chess_zero.lib.model_helper import load_best_model_weight
    model = ChessModel(config)
    if not load_best_model_weight(model):
        raise RuntimeError("Best model not found!")
    return ChessPlayer(config, model.get_pipes(config.play.search_threads))

print("Initializing chess model...")
default_config = Config()
PlayWithHumanConfig().update_play_config(default_config.play)
player = None
env = ChessEnv().reset()

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint that also verifies model status"""
    global player
    try:
        if player is None:
            player = get_player(default_config)
        
        return jsonify({
            "status": "healthy",
            "model_loaded": True,
            "board_state": str(env.board)
        })
    except Exception as e: 
        return jsonify({
            "status": "unhealthy",
            "error": str(e),
            "model_loaded": False
        })

@app.route('/predict', methods=['POST'])
def predict():
    """Make a move prediction based on current board state"""
    global player, env
    
    try:
        # Initialize player if needed
        if player is None:
            player = get_player(default_config)
        
        # Get AI's move recommendation
        action = player.action(env, False)
        
        # Get additional information similar to demo notebook
        board_before = str(env.board)
        
        # Create response with move and board state
        response = {
            'status': 'success',
            'move': action,
            'board_state': board_before,
        }
        
        return jsonify(response)
        
    except Exception as e:
        print(f"Error during prediction: {str(e)}")
        return jsonify({
            'status': 'error',
            'error': str(e)
        }), 500

@app.route('/update_board', methods=['POST'])
def update_board():
    """Update internal board state after a move is made"""
    global env
    
    try:
        data = request.json
        if 'move' not in data:
            return jsonify({'status': 'error', 'error': 'No move provided'}), 400
            
        # Apply the move to update internal state
        move = data['move']
        env.step(move)
        
        return jsonify({
            'status': 'success',
            'board_state': str(env.board)
        })
        
    except Exception as e:
        return jsonify({
            'status': 'error',
            'error': str(e)
        }), 500

@app.route('/reset', methods=['POST'])
def reset_board():
    """Reset the board to initial state"""
    global env, player
    try:
        env = ChessEnv().reset()
        player = None # we also trying to reset player as well
        return jsonify({
            'status': 'success',
            'board_state': str(env.board)
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'error': str(e)
        }), 500

if __name__ == '__main__':
    print("Starting chess model server on http://localhost:5000")
    app.run(host='0.0.0.0', port=5000)