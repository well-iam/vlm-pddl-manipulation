import logging
import json

logger = logging.getLogger(__name__)

class TaskExecutor:

    def __init__(self, gemini_planner, robot, max_steps=10):
        self.gemini_planner = gemini_planner
        self.robot = robot
        self.max_steps = max_steps # Limite di sicurezza per evitare loop infiniti in execute_autonomous_task

        #Stato interno del task
        self.last_action_feedback = "Nessuna azione precedente."
        self.current_plan = []
        self.world_state = None

    def execute(self, goal, strategy="planning"):
        """
        Metodo unificato per lanciare il task con la strategia scelta.
        """
        if strategy == "reactive":
            logger.info("Avvio esecuzione in modalità REATTIVA (Inner Monologue)...")
            return self.execute_task_step_by_step(goal)

        elif strategy == "planning":
            logger.info("Avvio esecuzione in modalità PIANIFICAZIONE (Plan-and-Execute)...")
            return self.execute_plan(goal)

        else:
            raise ValueError(f"Strategia '{strategy}' non supportata.")

    def _update_perception(self):
        """
        Metodo interno per aggiornare la percezione e salvare lo stato.
        Restituisce la stringa JSON pronta per il prompt.
        """
        self.robot.perception.perceive_scene(self.gemini_planner, use_vision=False)
        # 1. Ottieni il dizionario (lo salviamo per uso interno)
        # Nota: get_world_state_data restituisce un DIZIONARIO, non una stringa
        self.world_state = self.robot.perception.get_world_state_data()

        # 2. Converti in JSON (solo per il prompt)
        return json.dumps(self.world_state, indent=2)

    def _process_skill(self, skill_nome, argomenti):
        """
        Gestisce l'esecuzione di una skill (o meta-skill come 'done'/'ask'),
        inclusa la chiamata al robot e l'aggiornamento del feedback.

        Returns:
            tuple: (task_completed: bool, force_replan: bool)
                   - task_completed: True se il task è finito (successo o impossibile).
                   - force_replan: True se l'azione è fallita e serve ripianificare.
        """
        robot = self.robot

        # CASO 1: Il task è terminato (successo o fallimento dichiarato dal modello)
        if skill_nome == "done":
            logger.info(f"✅ Task completato (ricevuto 'done' dal planner).")
            return True, False

        # CASO 2: Il modello deve porre una domanda (Ambiguità)
        elif skill_nome == "ask_for_clarification":
            domanda_del_robot = argomenti[0] if argomenti else "Non ho capito, puoi specificare?"
            logger.info(f"🤖 ROBOT CHIEDE: {domanda_del_robot}")

            # Ottieni la risposta dall'utente
            risposta_utente = input("La tua risposta: ")

            self.last_action_feedback = f"""
                    Ho chiesto: '{domanda_del_robot}'. L'utente ha risposto: '{risposta_utente}'.
                    """
            return False, True

        # CASO 3: Esecuzione Fisica
        elif skill_nome in robot.available_skills:
            funzione_da_chiamare = robot.available_skills[skill_nome]
            success_detector = robot.available_detectors[skill_nome]
            try:
                successo_esecuzione = funzione_da_chiamare(*argomenti)

                # AUTO-FEEDBACK (Success Detector)
                if successo_esecuzione:
                    # Controlla se la skill è riuscita *davvero*
                    successo_reale, feedback_msg = success_detector(*argomenti)

                    if successo_reale:
                        self.last_action_feedback = f"Azione '{skill_nome}' eseguita con successo."
                        logger.info(f"  - {self.last_action_feedback}")
                        return False, False
                    else:
                        self.last_action_feedback = f"Azione '{skill_nome}' è fallita: {feedback_msg}"
                        logger.warning(f"  - {self.last_action_feedback} -> Richiesta ripianificazione.")
                        return False, True
                else:
                    self.last_action_feedback = f"AZIONE FALLITA (Esecuzione): La skill '{skill_nome}' ha riportato un fallimento."
                    logger.warning(f"  - {self.last_action_feedback} -> Richiesta ripianificazione.")
                    return False, True

            except Exception as e:
                logger.exception(f"❌ Errore critico nell'esecuzione di '{skill_nome}'.")
                self.last_action_feedback = f"Errore critico durante l'esecuzione: {e}"
                return False, True
        else:
            logger.error(f"Errore: Gemini ha scelto una skill non mappata: {skill_nome}")
            self.last_action_feedback = f"Errore di pianificazione: scelta skill '{skill_nome}' inesistente."
            return False, True

    def execute_task_step_by_step(self, high_level_goal):
        """
        Esegue l'INNER LOOP (Monologo Interiore) per un singolo obiettivo.
        """
        print(f"\n======= AVVIO TASK AUTONOMO: '{high_level_goal}' =======")
        gemini_planner = self.gemini_planner
        robot = self.robot

        # Stato iniziale per il loop
        self.last_action_feedback = "Nessuna azione precedente."

        task_completed = False
        step_counter = 0
        while not task_completed and step_counter < self.max_steps:
            step_counter += 1
            print(f"\n--- Turno {step_counter}/{self.max_steps} ---")

            # PERCEZIONE
            world_state_json = self._update_perception()

            # --- 3. PIANIFICAZIONE (Costruisci prompt di turno) ---
            prompt_di_turno = f"""
            **Stato Attuale del Mondo:**
            {world_state_json}

            **Feedback Azione Precedente:**
            {self.last_action_feedback}

            **Obiettivo Finale:**
            "{high_level_goal}"

            Genera il JSON per la prossima azione.
            """

            # PIANIFICAZIONE
            scelta_gemini = gemini_planner.ask_for_skill_json(prompt_di_turno, list(robot.available_skills.keys()))

            # --- 4. ESECUZIONE E FEEDBACK ---
            if not scelta_gemini:
                print("Pianificazione fallita: Gemini non ha fornito una risposta valida.")
                self.last_action_feedback = "Errore del planner: non ho ricevuto una risposta JSON valida."
                continue

            skill_nome = scelta_gemini["skill_scelta"]
            argomenti = scelta_gemini["argomenti"]
            # --- 6. ESEGUI LA SKILL ---
            task_completed, force_replan = self._process_skill(skill_nome, argomenti)

        # --- 8. CONDIZIONE DI TERMINAZIONE (FALLIMENTO/TIMEOUT) ---
        if task_completed:
            logger.info(f"✅ Task completato.")
            return True
        else:
            print(f"❌ Task fallito: Raggiunto limite massimo di {self.max_steps} passi.")
            return False

    def execute_plan(self, high_level_goal):
        """
        Esegue un task autonomo:
        1. Pianifica un piano completo.
        2. Esegue il piano passo-passo.
        3. Se un passo fallisce, si ferma e ripianifica.
        """
        logger.info(f"======= AVVIO TASK AUTONOMO: '{high_level_goal}' =======")
        gemini_client = self.gemini_planner

        # --- STATO DEL CICLO DI ESECUZIONE ---
        self.current_plan = []  # La lista di passi da eseguire
        self.last_action_feedback = "Nessuna azione precedente. Questo è il primo passo."
        task_completed = False

        # Avvia la chat. L'istruzione di sistema verrà formattata e inviata nel loop
        gemini_client.start_chat()  # Resetta la cronologia

        while not task_completed:
            # --- 1. FASE DI PIANIFICAZIONE O RIPIANIFICAZIONE ---
            if not self.current_plan:  # Se non abbiamo un piano, ne creiamo uno
                logger.info("Nessun piano attivo. Generazione di un nuovo piano...")

                # 1a. Percezione
                world_state_json = self._update_perception()

                # 1b. Formattazione Prompt
                # Inizializziamo l'istruzione di sistema con i dati di scena FRESCHI
                gemini_client.update_system_instruction(dynamic_world_state=world_state_json)

                # 1c. Richiesta Piano
                prompt_di_pianificazione = f"""
                **Feedback Azione Precedente:**
                {self.last_action_feedback}

                **Obiettivo Finale:**
                "{high_level_goal}"

                Genera il JSON completo del piano per raggiungere l'obiettivo.
                """

                # Chiediamo il JSON del piano
                risposta_gemini = gemini_client.send_chat_message(prompt_di_pianificazione)

                try:
                    # 1d. Parsing del Piano
                    json_text = risposta_gemini.text.strip().replace("```json", "").replace("```", "")
                    plan_data = json.loads(json_text)
                    self.current_plan = plan_data.get("piano", [])  # Estrae la lista di passi

                    if not self.current_plan:
                        logger.error("Pianificazione fallita: Gemini ha restituito un piano vuoto o malformato.")
                        task_completed = True  # Esce dal loop
                        continue

                    logger.info(f"Piano ricevuto con {len(self.current_plan)} passi.")

                except Exception as e:
                    logger.error(f"Pianificazione fallita: impossibile parsare il JSON di Gemini. {e}", exc_info=True)
                    task_completed = True  # Esce dal loop
                    continue

            # --- 2. FASE DI ESECUZIONE (esegue il prossimo passo del piano) ---
            # Prendi il prossimo passo dalla lista
            step = self.current_plan.pop(0)  # Estrae il primo elemento, accorciando la lista

            skill_nome = step.get("skill_scelta")
            argomenti = step.get("argomenti", [])

            logger.info(f"Esecuzione Step: {skill_nome}({', '.join(argomenti)})")
            logger.debug(f"  - Ragionamento del modello: {step.get('ragionamento')}")

            # --- 3. PROCESSAMENTO DELL'AZIONE ---
            task_completed, force_replan = self._process_skill(skill_nome, argomenti)
            if force_replan:
                logger.info("Rilevato fallimento o ambiguità. Abbandono il piano corrente e ripianifico.")
                self.current_plan = []  # Svuota il piano per attivare la logica di pianificazione al prossimo giro
                continue